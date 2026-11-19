"""Test doubles: a simulated clock and an in-memory radio with loss, delay and blackouts.

FakeLink has the same two methods as swarm.net.RadioLink: send(dst, mtype, **body) and
recv_all() -> list of message dicts, each with "t_recv" added.
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
    """Every message passes here. loss: drop probability per recipient; delay_s: latency."""

    def __init__(self, clock, loss=0.0, delay_s=0.0, seed=0):
        self.clock, self.loss, self.delay_s = clock, loss, delay_s
        self.rng = random.Random(seed)
        self.nodes = set()
        self.queue = []          # (delivery time, counter, dst, msg)
        self.counter = itertools.count()
        self.blackouts = []      # (node, t_from, t_to): the node neither sends nor receives

    def blacked_out(self, node, t) -> bool:
        return any(n == node and t_from <= t < t_to for n, t_from, t_to in self.blackouts)

    def link(self, node_id):
        self.nodes.add(node_id)
        return FakeLink(node_id, self)

    def submit(self, src, dst, msg):
        now = self.clock()
        if self.blacked_out(src, now):
            return
        targets = [n for n in self.nodes if n != src] if dst == "all" else [dst]
        for d in targets:
            if self.rng.random() >= self.loss:
                heapq.heappush(self.queue, (now + self.delay_s, next(self.counter), d, msg))

    def collect(self, node):
        now = self.clock()
        out, keep = [], []
        while self.queue and self.queue[0][0] <= now:
            item = heapq.heappop(self.queue)
            if item[2] != node:
                keep.append(item)
            elif not self.blacked_out(node, item[0]):
                out.append(dict(item[3], t_recv=now))
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
        msg = {"src": self.id, "dst": dst, "seq": self.seq, "t": self.radio.clock(),
               "type": mtype, **body}
        self.sent.append(msg)
        self.radio.submit(self.id, dst, msg)

    def recv_all(self):
        return self.radio.collect(self.id)
