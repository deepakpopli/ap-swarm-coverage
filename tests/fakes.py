"""Test doubles: simulated clock and an in-memory radio with loss, delay and blackouts.

FakeLink has the same two methods as swarm.net.RadioLink: send(dst, mtype, **body) and recv_all().
"""
import heapq
import itertools
import random


class SimClock:
    def __init__(self, t0: float = 1000.0):
        self.t = t0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class FakeRadio:
    """Every message passes here. loss: drop probability per recipient; delay: s."""

    def __init__(self, clock, loss=0.0, delay=0.0, seed=0):
        self.clock, self.loss, self.delay = clock, loss, delay
        self.rng = random.Random(seed)
        self.nodes = set()
        self.queue = []          # (deliver_t, counter, dst, msg)
        self._ctr = itertools.count()
        self.blackouts = []      # (node, t_from, t_to): node neither sends nor receives
        self.dead = set()

    def blacked_out(self, node, t) -> bool:
        return node in self.dead or any(n == node and a <= t < b for n, a, b in self.blackouts)

    def link(self, node_id):
        self.nodes.add(node_id)
        return FakeLink(node_id, self)

    def submit(self, src, dst, msg):
        now = self.clock()
        if self.blacked_out(src, now):
            return
        targets = [n for n in self.nodes if n != src] if dst == "all" else [dst]
        for d in targets:
            if d not in self.nodes or self.rng.random() < self.loss:
                continue
            heapq.heappush(self.queue, (now + self.delay, next(self._ctr), d, msg))

    def collect(self, node):
        now = self.clock()
        out, keep = [], []
        while self.queue and self.queue[0][0] <= now:
            t, c, d, msg = heapq.heappop(self.queue)
            if d == node:
                if not self.blacked_out(node, t):
                    out.append(dict(msg))
            else:
                keep.append((t, c, d, msg))
        for item in keep:
            heapq.heappush(self.queue, item)
        return out


class FakeLink:
    def __init__(self, node_id, radio: FakeRadio):
        self.id, self.radio = node_id, radio
        self.seq = 0
        self.sent = []

    def send(self, dst, mtype, **body):
        self.seq += 1
        msg = {"src": self.id, "dst": dst, "seq": self.seq, "t": self.radio.clock(), "type": mtype}
        msg.update(body)
        self.sent.append(msg)
        self.radio.submit(self.id, dst, msg)

    def recv_all(self):
        return self.radio.collect(self.id)
