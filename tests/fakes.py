"""Test doubles: a simulated clock and an in-memory radio with loss, delay and blackouts.

FakeLink has the same two methods as swarm.net.RadioLink: send(msg) and recv_all() -> list of
decoded MAVLink messages, each with .t_recv set. Packets are packed and decoded for real.
"""
import heapq
import itertools
import random

from swarm import protocol
from swarm import swarm_mav as mav


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
                out.append((item[3], now))
        for item in keep:
            heapq.heappush(self.queue, item)
        return out


class FakeLink:
    def __init__(self, node_id, radio: FakeRadio):
        self.id, self.radio = node_id, radio
        self.mav = mav.MAVLink(None, srcSystem=node_id, srcComponent=protocol.COMPONENT)
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)
        dst = protocol.destination(msg)
        self.radio.submit(self.id, "all" if dst is None else dst, bytes(msg.pack(self.mav)))

    def recv_all(self):
        out = []
        for data, t_recv in self.radio.collect(self.id):
            msg = self.mav.decode(bytearray(data))
            msg.t_recv = t_recv
            out.append(msg)
        return out
