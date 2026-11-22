"""Radio emulator: one UDP relay that every message between nodes passes through.

    python -m sim.radio --n N --seed S --loss 0.2 --delay 0.2 --port-base 0 --log run/radio.jsonl
                        [--blackout ID:T0:T1 ...] [--mute ID:T0:T1 ...]

Each datagram is one MAVLink packet from the header's source system, to target_system for
SWARM_ASSIGN and SWARM_INFEASIBLE, else to every other node; nodes listen on net.listen_port(id),
so nothing registers. Per receiver a copy is dropped with probability `loss`, else forwarded
`delay` s after arrival. --blackout drops all to and from ID in mission time [T0, T1); --mute
only what ID sends (it still hears). Mission time comes from the ground's SWARM_START.

Determinism: one random.Random per (src, dst), one draw per copy in arrival order, so the k-th
copy from A to B is dropped in every run or in none (MAVLink seq would wrap every 256).
The queue holds at most `delay` s of traffic: each pass reads one datagram and forwards all due.

Log (JSON lines, "t_wall", "event"): "config", "start_seen", and at exit "stats" per "src->dst"
and in total {sent, delivered, dropped_loss, dropped_window, delay_mean_s, delay_max_s};
delay = forward time - receive time.
"""
import argparse
import heapq
import json
import random
import select
import signal
import socket
import time
from dataclasses import dataclass

from swarm import config, protocol
from swarm import swarm_mav as mav
from swarm.net import HOST, listen_port


def pair_rng(seed, src, dst) -> random.Random:
    """The loss draws of one (src, dst) pair; a str seed is hashed the same in every process."""
    return random.Random(f"{seed}:{src}:{dst}")


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
    def __init__(self, n, seed, loss, delay_s, port_base, log_path, blackouts, mutes):
        self.nodes = [config.GROUND_ID] + list(range(1, n + 1))
        self.seed = seed
        self.loss = loss
        self.delay_s = delay_s
        self.port_base = port_base
        self.blackouts = blackouts
        self.mutes = mutes
        self.parser = mav.MAVLink(None)
        self.rngs = {}             # (src, dst) -> random.Random, see pair_rng
        self.t0 = None             # mission t0 (wall time), from the ground's SWARM_START
        self.queue = []            # heap of (due time, count, data, src, dst, receive time)
        self.count = 0             # tie-breaker: copies due at the same time keep their order
        self.stats = {}            # "src->dst" -> PairStats
        self.log = open(log_path, "a")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((HOST, config.RADIO_PORT + port_base))
        self.write("config", n=n, seed=seed, loss=loss, delay_s=delay_s, port_base=port_base,
                   blackouts=blackouts, mutes=mutes)

    def write(self, event, **fields):
        self.log.write(json.dumps({"t_wall": time.time(), "event": event, **fields}) + "\n")
        self.log.flush()

    def on_datagram(self, data, now):
        try:
            msg = self.parser.decode(bytearray(data))
        except mav.MAVError:
            return             # not one of our packets
        src = msg.get_srcSystem()
        if msg.get_type() == "SWARM_START" and msg.t0_usec / 1e6 != self.t0:
            self.t0 = msg.t0_usec / 1e6
            self.write("start_seen", t0=self.t0)
        dst = protocol.destination(msg)
        for receiver in [dst] if dst is not None else [d for d in self.nodes if d != src]:
            self.route(data, src, receiver, now)

    def route(self, data, src, dst, now):
        stats = self.stats.setdefault(f"{src}->{dst}", PairStats())
        stats.sent += 1
        rng = self.rngs.setdefault((src, dst), pair_rng(self.seed, src, dst))
        lost = rng.random() < self.loss      # drawn for every copy: windows do not shift it
        if self.t0 is not None and window_drop(src, dst, now - self.t0, self.blackouts,
                                               self.mutes):
            stats.dropped_window += 1
        elif lost:
            stats.dropped_loss += 1
        else:
            self.count += 1
            heapq.heappush(self.queue, (now + self.delay_s, self.count, data, src, dst, now))

    def forward_due(self, now):
        while self.queue and self.queue[0][0] <= now:
            _, _, data, src, dst, t_recv = heapq.heappop(self.queue)
            try:
                self.sock.sendto(data, (HOST, listen_port(dst, self.port_base)))
            except OSError:
                continue
            delay = time.time() - t_recv
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
    ap.add_argument("--n", type=int, required=True, help="agents 1..N")
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
    Radio(args.n, args.seed, args.loss, args.delay, args.port_base, args.log, args.blackout,
          args.mute).run()


if __name__ == "__main__":
    main()
