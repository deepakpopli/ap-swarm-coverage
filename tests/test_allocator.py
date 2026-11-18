"""Unit tests for swarm/allocator.py (pure, fast)."""
import math
import unittest

from swarm import config
from swarm.allocator import AllocParams, allocate, compute_strips, slot_key

P = AllocParams()


def start_positions(n):
    return {i + 1: config.start_xy(i, n) for i in range(n)}


def legs(asg, pos=None):
    """All legs an agent flies: pos -> slot0, slot i -> i+1, last -> slots[loop_from]."""
    pts = asg.slots
    out = list(zip(pts, pts[1:])) + [(pts[-1], pts[asg.loop_from])]
    if pos is not None:
        out.insert(0, (list(pos[:2]), pts[0]))
    return out


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class TestAllocator(unittest.TestCase):
    def test_determinism(self):
        pos = start_positions(3)
        bl = {(2, slot_key((150.0, 109.5)))}
        a = allocate([3, 1, 2], pos, bl, P)
        b = allocate([2, 3, 1], dict(reversed(list(pos.items()))), set(bl), P)
        self.assertEqual({k: v.to_dict() for k, v in a.items()},
                         {k: v.to_dict() for k, v in b.items()})
        self.assertEqual(repr(a), repr(b))

    def test_strips_tile_area(self):
        for n in range(1, 7):
            res = allocate(range(1, n + 1), start_positions(n), set(), P)
            shares = sorted(r.share for r in res.values())
            xmin, xmax, ymin, ymax = config.AREA
            self.assertEqual(shares[0][0], xmin)
            self.assertEqual(shares[-1][1], xmax)
            for s in shares:
                self.assertEqual((s[2], s[3]), (ymin, ymax))
                self.assertGreater(s[1], s[0])
            for left, right in zip(shares, shares[1:]):
                self.assertEqual(left[1], right[0])     # shared edge: no gap, no overlap
            self.assertAlmostEqual(sum(s[1] - s[0] for s in shares), xmax - xmin, places=9)

    def test_strips_left_to_right_by_x(self):
        pos = {1: (280.0, 0.0), 2: (10.0, 0.0), 3: (150.0, 50.0), 4: (150.0, 0.0)}
        res = allocate([1, 2, 3, 4], pos, set(), P)
        order = sorted(res, key=lambda a: res[a].share[0])
        self.assertEqual(order, [2, 3, 4, 1])           # tie at x=150 broken by id

    def test_spacing_bound_and_loop_closure(self):
        for n in (1, 2, 3, 4, 5):
            for pos in (start_positions(n), {i: (299.0, 299.0) for i in range(1, n + 1)}):
                res = allocate(range(1, n + 1), pos, set(), P)
                for a, asg in res.items():
                    self.assertTrue(0 <= asg.loop_from < len(asg.slots))
                    for p, q in legs(asg, pos[a]):
                        self.assertLessEqual(dist(p, q), P.max_slot_spacing + 1e-9, (n, a, p, q))
                    # loop slots stay inside the share
                    s = asg.share
                    for x, y in asg.slots[asg.loop_from:]:
                        self.assertTrue(s[0] <= x <= s[1] and s[2] <= y <= s[3])

    def test_lanes_from_swath(self):
        res = allocate([1, 2, 3], start_positions(3), set(), P)
        # 100 m strip, 102 m swath -> one lane at the strip centre; ends inset by 57/2
        loop = res[2].slots[res[2].loop_from:]
        self.assertEqual({p[0] for p in loop}, {150.0})
        self.assertEqual(min(p[1] for p in loop), 28.5)
        self.assertEqual(max(p[1] for p in loop), 271.5)
        res = allocate([1, 3], start_positions(3), set(), P)
        loop = res[1].slots[res[1].loop_from:]
        self.assertEqual(sorted({p[0] for p in loop}), [37.5, 112.5])   # ceil(150/102) = 2 lanes

    def test_three_to_two(self):
        pos = start_positions(3)
        res = allocate([1, 3], pos, set(), P)
        self.assertEqual(sorted(res), [1, 3])
        self.assertEqual(res[1].share, [0.0, 150.0, 0.0, 300.0])
        self.assertEqual(res[3].share, [150.0, 300.0, 0.0, 300.0])

    def test_weights(self):
        params = AllocParams(weights=(1.0, 2.0, 1.0))
        res = allocate([1, 2, 3], start_positions(3), set(), params)
        self.assertEqual(res[1].share, [0.0, 75.0, 0.0, 300.0])
        self.assertEqual(res[2].share, [75.0, 225.0, 0.0, 300.0])
        self.assertEqual(res[3].share, [225.0, 300.0, 0.0, 300.0])
        with self.assertRaises(ValueError):
            compute_strips(2, config.AREA, (1.0, 2.0, 3.0))

    def _check_never_issued(self, alive, pos, bl):
        res = allocate(alive, pos, bl, P)
        for a, asg in res.items():
            for p in asg.slots:
                self.assertNotIn((a, slot_key(p)), bl, (a, p, asg.note))
            self.assertNotIn("degraded", asg.note)
            for p, q in legs(asg, pos[a]):
                self.assertLessEqual(dist(p, q), P.max_slot_spacing + 1e-9)
        return res

    def test_blacklisted_loop_slot_never_issued(self):
        pos = start_positions(3)
        base = allocate([1, 2, 3], pos, set(), P)
        for a in (1, 2, 3):
            for p in base[a].slots:                        # every slot, one at a time
                bl = {(a, slot_key(p))}
                res = self._check_never_issued([1, 2, 3], pos, bl)
                self.assertTrue(res[a].note, "a blacklisted slot must change something")

    def test_blacklist_many_slots(self):
        pos = start_positions(3)
        base = allocate([1, 2, 3], pos, set(), P)
        bl = {(2, slot_key(p)) for p in base[2].slots}      # the whole default plan of agent 2
        self._check_never_issued([1, 2, 3], pos, bl)

    def test_blacklist_only_affects_that_agent(self):
        pos = start_positions(3)
        base = allocate([1, 2, 3], pos, set(), P)
        bl = {(1, slot_key(base[2].slots[2]))}            # agent 1 never flies there anyway
        res = allocate([1, 2, 3], pos, bl, P)
        for a in (1, 2, 3):
            self.assertEqual(res[a].to_dict(), base[a].to_dict())

    def test_infeasible_escalates_to_swap(self):
        # Blacklist, for agent 2, every point of the middle strip's coverage area (nudges
        # included) but not the south edge band it transits along: it must take a neighbour's
        # strip, and the neighbour gets the middle strip.
        pos = start_positions(3)
        bl = {(2, (x, y)) for x in range(95, 206) for y in range(20, 281)}
        res = self._check_never_issued([1, 2, 3], pos, bl)
        self.assertIn("swapped", res[2].note)
        self.assertNotEqual(res[2].share, [100.0, 200.0, 0.0, 300.0])


if __name__ == "__main__":
    unittest.main()
