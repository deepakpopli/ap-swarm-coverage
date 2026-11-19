"""Radio emulator: one UDP relay that every message between nodes passes through.

    python -m sim.radio --seed S --loss 0.2 --delay 0.2 --port-base 0 --log run/radio.jsonl
                        [--blackout ID:T0:T1 ...] [--mute ID:T0:T1 ...]

HELLO {"port"} registers a node; nodes repeat it every second, so a restarted radio relearns
them. Any other message goes to each receiver (dst, or every registered node but src for "all"):
per receiver, dropped with probability `loss`, else forwarded unchanged after `delay` seconds.
--blackout drops everything to and from agent ID in mission time [T0, T1); --mute drops only
what ID sends (a one-way failure: it still hears). Mission time needs t0, which the radio
learns from the ground's START: the only place it reads inside a payload.

Determinism: the drop of one copy is a hash of (seed, src, dst, seq), so with the same seed
the k-th message from A to B is dropped in every run or in none. Not fixed by the seed: which
message gets seq k, what falls inside a window, and whether a receiver had registered in time.
More in docs/allocation.md.

Queue bound: each pass reads one datagram and forwards every copy that is due, so no copy
waits longer than `delay` plus one pass. The queue therefore holds at most what the radio read
in the last `delay` seconds; senders faster than the radio reads lose datagrams in the kernel's
fixed-size socket buffer, never in this queue.

Log (JSON lines, each with "t_wall" and "event"): "config" at start, "register", "start_seen",
and "stats" once at exit: per "src->dst" pair and in total {sent, delivered, dropped_loss,
dropped_window, delay_mean_s, delay_max_s}. sent - delivered - dropped = copies with no
registered receiver, a failed sendto, or still queued at exit. Delay = forward time - the
sender's "t".
"""
import argparse
import hashlib
import heapq
import json
import select
import signal
import socket
import time
from dataclasses import dataclass

from swarm import config, protocol

HOST = "127.0.0.1"


def drop_draw(seed, src, dst, seq) -> float:
    """A uniform number in [0, 1) that depends only on (seed, src, dst, seq)."""
    digest = hashlib.blake2b(f"{seed}:{src}:{dst}:{seq}".encode(), digest_size=8).digest()
    return int.from_bytes(digest, "big") / 2.0 ** 64


def in_window(windows, node, t_mission) -> bool:
    return any(node == w_id and t_start <= t_mission < t_end for w_id, t_start, t_end in windows)


def window_drop(src, dst, t_mission, blackouts, mutes) -> bool:
    """True if a blackout of src or dst, or a mute of src, covers this mission time."""
    return (in_window(blackouts, src, t_mission) or in_window(blackouts, dst, t_mission)
            or in_window(mutes, src, t_mission))


def parse_window(text) -> tuple:
    """'ID:T0:T1' -> (id, t0, t1), times in mission seconds."""
    node, t_start, t_end = text.split(":")
    return (int(node), float(t_start), float(t_end))


@dataclass
class PairStats:
    sent: int = 0
    delivered: int = 0
    dropped_loss: int = 0
    dropped_window: int = 0
    delay_sum_s: float = 0.0
    delay_max_s: float = 0.0

    def add(self, other):
        self.sent += other.sent
        self.delivered += other.delivered
        self.dropped_loss += other.dropped_loss
        self.dropped_window += other.dropped_window
        self.delay_sum_s += other.delay_sum_s
        self.delay_max_s = max(self.delay_max_s, other.delay_max_s)

    def to_dict(self) -> dict:
        mean = self.delay_sum_s / self.delivered if self.delivered else None
        return {"sent": self.sent, "delivered": self.delivered,
                "dropped_loss": self.dropped_loss, "dropped_window": self.dropped_window,
                "delay_mean_s": mean, "delay_max_s": self.delay_max_s}


class Radio:
    def __init__(self, seed, loss, delay_s, port_base, log_path, blackouts, mutes):
        self.seed = seed
        self.loss = loss
        self.delay_s = delay_s
        self.blackouts = blackouts
        self.mutes = mutes
        self.nodes = {}            # node id -> listen port
        self.t0 = None             # mission t0 (wall time), from the ground's START
        self.queue = []            # heap of (due time, count, data, src, dst, sender's t)
        self.count = 0             # tie-breaker: copies due at the same time keep their order
        self.stats = {}            # "src->dst" -> PairStats
        self.log = open(log_path, "a")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((HOST, config.RADIO_PORT + port_base))
        self.write("config", seed=seed, loss=loss, delay_s=delay_s, port_base=port_base,
                   blackouts=blackouts, mutes=mutes)

    def write(self, event, **fields):
        self.log.write(json.dumps({"t_wall": time.time(), "event": event, **fields}) + "\n")
        self.log.flush()

    def on_datagram(self, data, now):
        msg = protocol.decode(data)
        src = msg["src"]
        if msg["type"] == "HELLO":
            if self.nodes.get(src) != msg["port"]:
                self.nodes[src] = msg["port"]
                self.write("register", node=src, port=msg["port"])
            return
        if msg["type"] == "START" and msg["t0"] != self.t0:   # the one payload read
            self.t0 = msg["t0"]
            self.write("start_seen", t0=self.t0)
        receivers = [msg["dst"]] if msg["dst"] != "all" else sorted(set(self.nodes) - {src})
        for dst in receivers:
            self.route(data, src, dst, msg["seq"], msg["t"], now)

    def route(self, data, src, dst, seq, t_sent, now):
        stats = self.stats.setdefault(f"{src}->{dst}", PairStats())
        stats.sent += 1
        if self.t0 is not None and window_drop(src, dst, now - self.t0, self.blackouts,
                                               self.mutes):
            stats.dropped_window += 1
        elif drop_draw(self.seed, src, dst, seq) < self.loss:
            stats.dropped_loss += 1
        else:
            self.count += 1
            heapq.heappush(self.queue, (now + self.delay_s, self.count, data, src, dst, t_sent))

    def forward_due(self, now):
        while self.queue and self.queue[0][0] <= now:
            _, _, data, src, dst, t_sent = heapq.heappop(self.queue)
            if dst not in self.nodes:
                continue
            try:
                self.sock.sendto(data, (HOST, self.nodes[dst]))
            except OSError:
                continue
            delay = time.time() - t_sent
            stats = self.stats[f"{src}->{dst}"]
            stats.delivered += 1
            stats.delay_sum_s += delay
            stats.delay_max_s = max(stats.delay_max_s, delay)

    def run(self):
        try:
            while True:
                wait_s = None if not self.queue else max(0.0, self.queue[0][0] - time.time())
                readable, _, _ = select.select([self.sock], [], [], wait_s)
                if readable:
                    self.on_datagram(self.sock.recv(65535), time.time())
                self.forward_due(time.time())
        finally:
            self.write_stats()
            self.log.close()

    def write_stats(self):
        total = PairStats()
        for pair in self.stats.values():
            total.add(pair)
        self.write("stats", pairs={k: v.to_dict() for k, v in sorted(self.stats.items())},
                   total=total.to_dict())


def _exit_on_signal(signum, frame):
    raise SystemExit(0)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Radio emulator")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--loss", type=float, default=config.RADIO_LOSS)
    ap.add_argument("--delay", type=float, default=config.RADIO_DELAY_S)
    ap.add_argument("--port-base", type=int, default=0)
    ap.add_argument("--log", required=True)
    ap.add_argument("--blackout", type=parse_window, action="append", default=[])
    ap.add_argument("--mute", type=parse_window, action="append", default=[])
    args = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, _exit_on_signal)
    signal.signal(signal.SIGINT, _exit_on_signal)
    Radio(args.seed, args.loss, args.delay, args.port_base, args.log, args.blackout,
          args.mute).run()


if __name__ == "__main__":
    main()
