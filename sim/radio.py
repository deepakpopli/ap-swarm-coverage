"""Radio emulator: every node-to-node message passes through here.

    python -m swarm.radio --seed S --loss 0.2 --delay 0.2 --port-base 0 \
        --log run_dir/radio.jsonl [--blackout ID:T_START:T_END ...]

One UDP socket on 127.0.0.1:(RADIO_PORT + port_base).

* HELLO {"src": id, "port": p} registers (or re-registers) node `id` at 127.0.0.1:p.
  Nodes re-send HELLO every second, so a restarted radio relearns everyone within ~1 s.
  HELLO is never forwarded and never counted.
* Every other datagram is parsed only for "src", "dst", "seq", "t" (envelope). The receivers are
  [dst] for a numbered dst, or every registered node except src for dst "all". For each receiver
  independently: dropped by blackout, else dropped with probability `loss`, else forwarded
  unchanged (same bytes) after `delay` seconds.
* Blackout (Mission C): `--blackout ID:T0:T1` drops every message to AND from agent ID whose
  arrival at the radio falls in mission time [T0, T1). Mission t0 is learned by sniffing the
  ground station's START message ({"t0": wall time}); this is the ONLY place the radio looks
  inside a payload. Until a START has been seen no blackout is active. The latest START wins
  (a restarted ground that re-sends START with the same t0 changes nothing).

Determinism (what --seed does and does not fix)
  The drop decision for one (src, dst, seq) triple is a pure function:
      u = blake2b(f"{seed}:{src}:{dst}:{seq}") mapped to [0, 1);  dropped  <=>  u < loss
  It does not depend on arrival order, timing, or what other packets exist. So, for a given
  seed, "the k-th data message node 2 sends, as seen by node 3" is always dropped or always
  delivered, in every run.
  NOT deterministic (real time decides it): which message content gets seq k (e.g. whether the
  k-th message of an agent is a STATE or an INFEASIBLE), how many messages a node sends before
  some event, whether a receiver had registered (HELLO) when a broadcast arrived, which packets
  fall inside a blackout window, and the exact delivery times (delay is a timer, jitter ~1 ms
  plus OS scheduling). The overall drop rate per pair therefore matches `loss` statistically,
  and the exact set of dropped messages is reproducible only up to that timing.

Log (JSON lines, one object per line, every line has "t_wall" and "event"):
  {"event": "config", seed, loss, delay, port, blackout: [[id, t0, t1], ...]}
  {"event": "register", "node": id, "port": p}                 first HELLO or changed port
  {"event": "start_seen", "t0": float}                          START sniffed (t0 changed)
  {"event": "stats", "final": bool, "pairs": {"src->dst": {sent, delivered, dropped_loss,
        dropped_blackout, no_route, delay_mean, delay_max}}, "total": {same keys}}
     sent = per-receiver copies offered (a broadcast to 3 receivers counts 3).
     delay = (wall time the radio forwarded the copy) - (sender's envelope "t"), seconds,
             i.e. measured sender->radio-out latency, over delivered copies.
     no_route = receiver id never registered (not a loss; nobody was listening).
  "stats" is written every --stats-every seconds (cumulative) and once more at exit (final).
"""
import argparse
import hashlib
import heapq
import json
import select
import signal
import socket
import sys
import time

from swarm import config

HOST = "127.0.0.1"


def drop_draw(seed, src, dst, seq):
    """Deterministic uniform number in [0, 1) for one (src, dst, seq) copy."""
    h = hashlib.blake2b(f"{seed}:{src}:{dst}:{seq}".encode(), digest_size=8).digest()
    return int.from_bytes(h, "big") / 2.0 ** 64


def parse_blackout(text):
    """'ID:T_START:T_END' -> (id, t_start, t_end) in mission seconds."""
    parts = text.split(":")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("blackout must be ID:T_START:T_END")
    return (int(parts[0]), float(parts[1]), float(parts[2]))


class PairStats:
    def __init__(self):
        self.sent = 0
        self.delivered = 0
        self.dropped_loss = 0
        self.dropped_blackout = 0
        self.no_route = 0
        self.delay_sum = 0.0
        self.delay_max = 0.0

    def add_delivered(self, delay):
        self.delivered += 1
        self.delay_sum += delay
        self.delay_max = max(self.delay_max, delay)

    def merge(self, other):
        for k in ("sent", "delivered", "dropped_loss", "dropped_blackout", "no_route", "delay_sum"):
            setattr(self, k, getattr(self, k) + getattr(other, k))
        self.delay_max = max(self.delay_max, other.delay_max)

    def to_dict(self):
        mean = self.delay_sum / self.delivered if self.delivered else None
        return {"sent": self.sent, "delivered": self.delivered,
                "dropped_loss": self.dropped_loss, "dropped_blackout": self.dropped_blackout,
                "no_route": self.no_route,
                "delay_mean": round(mean, 4) if mean is not None else None,
                "delay_max": round(self.delay_max, 4) if self.delivered else None}


class Radio:
    def __init__(self, seed, loss, delay, port_base, log_path, blackouts=(), stats_every=5.0):
        self.seed = seed
        self.loss = loss
        self.delay = delay
        self.blackouts = list(blackouts)
        self.stats_every = stats_every
        self.port = config.RADIO_PORT + port_base
        self.nodes = {}          # node id -> listen port
        self.t0 = None           # mission t0 (wall), from the ground's START
        self.queue = []          # heap of (due_wall, counter, data, port, pair_key, t_sent)
        self.counter = 0
        self.stats = {}          # "src->dst" -> PairStats
        self.log = open(log_path, "a") if log_path else None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((HOST, self.port))
        self.sock.setblocking(False)
        self.write({"event": "config", "seed": seed, "loss": loss, "delay": delay,
                    "port": self.port, "blackout": [list(b) for b in self.blackouts]})

    # ---- logging
    def write(self, rec):
        if self.log is None:
            return
        rec = dict(rec, t_wall=time.time())
        self.log.write(json.dumps(rec) + "\n")
        self.log.flush()

    def write_stats(self, final):
        total = PairStats()
        for s in self.stats.values():
            total.merge(s)
        self.write({"event": "stats", "final": final,
                    "pairs": {k: self.stats[k].to_dict() for k in sorted(self.stats)},
                    "total": total.to_dict()})

    # ---- decisions
    def blacked_out(self, node, now):
        if self.t0 is None:
            return False
        t_mission = now - self.t0
        return any(node == bid and t_start <= t_mission < t_end
                   for bid, t_start, t_end in self.blackouts)

    def receivers(self, src, dst):
        if dst == "all":
            return sorted(n for n in self.nodes if n != src)
        return [dst]

    # ---- packet handling
    def on_datagram(self, data, now):
        try:
            msg = json.loads(data.decode())
            src, dst, mtype = msg["src"], msg["dst"], msg["type"]
        except (ValueError, KeyError, TypeError, UnicodeDecodeError):
            return
        if mtype == "HELLO":
            port = int(msg.get("port", 0))
            if self.nodes.get(src) != port:
                self.nodes[src] = port
                self.write({"event": "register", "node": src, "port": port})
            return
        if mtype == "START" and src == config.GROUND_ID:   # the one payload peek (see docstring)
            t0 = float(msg.get("t0"))
            if t0 != self.t0:
                self.t0 = t0
                self.write({"event": "start_seen", "t0": t0})
        seq = msg.get("seq", 0)
        t_sent = msg.get("t", now)
        for rx in self.receivers(src, dst):
            self.route_copy(data, src, rx, seq, t_sent, now)

    def route_copy(self, data, src, rx, seq, t_sent, now):
        key = f"{src}->{rx}"
        st = self.stats.setdefault(key, PairStats())
        st.sent += 1
        if self.blacked_out(src, now) or self.blacked_out(rx, now):
            st.dropped_blackout += 1
            return
        if drop_draw(self.seed, src, rx, seq) < self.loss:
            st.dropped_loss += 1
            return
        port = self.nodes.get(rx)
        if port is None:
            st.no_route += 1
            return
        self.counter += 1
        heapq.heappush(self.queue, (now + self.delay, self.counter, data, port, key, t_sent))

    def flush_due(self, now):
        while self.queue and self.queue[0][0] <= now:
            _, _, data, port, key, t_sent = heapq.heappop(self.queue)
            try:
                self.sock.sendto(data, (HOST, port))
            except OSError:
                pass
            self.stats[key].add_delivered(time.time() - t_sent)

    # ---- main loop
    def run(self):
        next_stats = time.time() + self.stats_every
        try:
            while True:
                now = time.time()
                wait = 0.05
                if self.queue:
                    wait = min(wait, max(0.0, self.queue[0][0] - now))
                ready, _, _ = select.select([self.sock], [], [], wait)
                if ready:
                    while True:
                        try:
                            data, _ = self.sock.recvfrom(65535)
                        except (BlockingIOError, InterruptedError):
                            break
                        except OSError:
                            continue
                        self.on_datagram(data, time.time())
                now = time.time()
                self.flush_due(now)
                if now >= next_stats:
                    self.write_stats(final=False)
                    next_stats = now + self.stats_every
        finally:
            self.write_stats(final=True)
            if self.log:
                self.log.close()


def _exit_on_signal(signum, frame):
    raise SystemExit(0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--loss", type=float, default=config.RADIO_LOSS)
    ap.add_argument("--delay", type=float, default=config.RADIO_DELAY_S)
    ap.add_argument("--port-base", type=int, default=0)
    ap.add_argument("--log", default=None)
    ap.add_argument("--blackout", type=parse_blackout, action="append", default=[])
    ap.add_argument("--stats-every", type=float, default=5.0)
    args = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, _exit_on_signal)
    signal.signal(signal.SIGINT, _exit_on_signal)
    Radio(args.seed, args.loss, args.delay, args.port_base, args.log,
          args.blackout, args.stats_every).run()


if __name__ == "__main__":
    sys.exit(main())
