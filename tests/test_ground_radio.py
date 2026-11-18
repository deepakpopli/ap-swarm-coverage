"""Integration test: radio + ground as real subprocesses, driven by 3 fake agent stubs.

The stubs are plain RadioLink objects that broadcast STATE at STATE_HZ and record what they
receive. No simulator. Shortened timeouts are passed on the ground's command line
(config defaults are unchanged). Takes ~40 s of wall time, very little CPU.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import unittest

from swarm import config
from swarm.allocator import slot_key
from swarm.net import RadioLink

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SUSPECT_S, LOST_S, REPLAN_MIN_S = 1.0, 3.0, 2.0


def free_port_base(offset):
    """Ports 24000..28200, below the Linux ephemeral range; varies with pid to avoid clashes."""
    return 10000 + (os.getpid() % 200) * 20 + offset


def spawn(args, log_name):
    return subprocess.Popen([sys.executable, "-m"] + args, cwd=REPO,
                            stdout=subprocess.DEVNULL, stderr=open(log_name + ".err", "w"))


def stop(proc):
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()


def read_jsonl(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


class Stub:
    """A fake agent: sends STATE, remembers what it sent and received."""

    def __init__(self, agent_id, port_base, pos, rate=config.STATE_HZ):
        self.id = agent_id
        self.link = RadioLink(agent_id, port_base)
        self.pos = list(pos)
        self.rate = rate
        self.status = "ready"
        self.epoch = 0
        self.alive = True
        self.next_state = 0.0
        self.state_sent = []          # (seq, t_send)
        self.inbox = []
        self.assign = None            # latest ASSIGN (highest epoch)
        self.t0 = None

    def pump(self, now):
        if not self.alive:
            return
        if now >= self.next_state:
            seq = self.link.send("all", "STATE", pos=self.pos + [60.0], vel=[0, 0, 0],
                                 epoch=self.epoch, slot=0, status=self.status)
            self.state_sent.append((seq, now))
            self.next_state = now + 1.0 / self.rate
        for m in self.link.recv_all():
            self.inbox.append(m)
            if m["type"] == "START":
                self.t0 = m["t0"]
            if m["type"] == "ASSIGN" and (self.assign is None or m["epoch"] > self.assign["epoch"]):
                self.assign = m
                self.epoch = m["epoch"]

    def kill(self):
        self.alive = False
        self.link.close()


def run_for(stubs, seconds, until=None):
    end = time.time() + seconds
    while time.time() < end:
        now = time.time()
        for s in stubs:
            s.pump(now)
        if until is not None and until():
            return True
        time.sleep(0.005)
    return until is None


def link_stats(stubs, since, until):
    """Measured loss and delay of STATE between stubs, for messages sent in [since, until)."""
    sent = recv = 0
    delays = []
    for tx in stubs:
        seqs = {seq for seq, t in tx.state_sent if since <= t < until}
        for rx in stubs:
            if rx is tx:
                continue
            got = [m for m in rx.inbox if m["type"] == "STATE" and m["src"] == tx.id
                   and m["seq"] in seqs]
            sent += len(seqs)
            recv += len(got)
            delays += [m["t_recv"] - m["t"] for m in got]
    return sent, 1.0 - recv / sent, delays


class TestGroundRadio(unittest.TestCase):
    def test_full_scenario(self):
        pb = free_port_base(0)
        tmp = tempfile.mkdtemp(prefix="swarm_gr_")
        radio_log, ground_log = os.path.join(tmp, "radio.jsonl"), os.path.join(tmp, "ground.jsonl")
        radio = spawn(["swarm.radio", "--seed", "7", "--loss", "0.2", "--delay", "0.2",
                       "--port-base", str(pb), "--log", radio_log, "--stats-every", "2"], radio_log)
        ground = spawn(["swarm.ground", "--n", "3", "--port-base", str(pb), "--log", ground_log,
                        "--mission-s", "120", "--suspect-s", str(SUSPECT_S),
                        "--lost-s", str(LOST_S), "--replan-min-s", str(REPLAN_MIN_S)], ground_log)
        stubs = []
        try:
            time.sleep(0.5)
            stubs = [Stub(i + 1, pb, config.start_xy(i, 3)) for i in range(3)]

            # 1. START and ASSIGN reach every stub; first epoch has three 100 m strips
            ok = run_for(stubs, 15, lambda: all(s.t0 and s.assign for s in stubs))
            self.assertTrue(ok, "START/ASSIGN did not arrive")
            for s in stubs:
                s.status = "cover"
                self.assertEqual(s.assign["share"][1] - s.assign["share"][0], 100.0)
                self.assertEqual(s.assign["alive"], [1, 2, 3])

            # 2. measured loss and latency over a few hundred stub->stub copies
            t_a = time.time()
            run_for(stubs, 12)
            t_b = time.time()
            run_for(stubs, 0.6)        # let in-flight messages land
            n, loss, delays = link_stats(stubs, t_a + 0.0, t_b)
            mean_d = sum(delays) / len(delays)
            print("\nmeasured stub->stub: copies=%d loss=%.3f delay mean=%.3f min=%.3f max=%.3f s"
                  % (n, loss, mean_d, min(delays), max(delays)))
            self.assertGreater(n, 300)
            self.assertTrue(0.13 < loss < 0.27, loss)
            self.assertGreaterEqual(min(delays), 0.195)
            self.assertTrue(0.195 < mean_d < 0.25, mean_d)

            # 3. agent 2 dies -> re-plan to two 150 m strips after ~LOST_S
            epoch_before = stubs[0].epoch
            stubs[1].kill()
            t_kill = time.time()
            survivors = [stubs[0], stubs[2]]
            ok = run_for(survivors, LOST_S + 6, lambda: all(
                s.assign["alive"] == [1, 3] for s in survivors))
            t_seen = time.time()
            self.assertTrue(ok, "no re-plan after agent loss")
            print("re-plan seen by both survivors %.2f s after the kill (LOST_S=%.1f)"
                  % (t_seen - t_kill, LOST_S))
            self.assertGreaterEqual(t_seen - t_kill, LOST_S)
            for s in survivors:
                self.assertGreater(s.epoch, epoch_before)
                self.assertEqual(s.assign["share"][1] - s.assign["share"][0], 150.0)

            # 4. INFEASIBLE for the current epoch -> new epoch without that (agent, slot) pair
            s1 = stubs[0]
            e_bad, asg = s1.epoch, s1.assign
            idx = asg["loop_from"]
            bad_key = slot_key(asg["slots"][idx])
            body = dict(epoch=e_bad, slot=idx, reason="timeout", pos=s1.pos + [60.0])
            deadline = time.time() + 8
            while s1.epoch == e_bad and time.time() < deadline:   # repeated until new epoch
                s1.link.send(0, "INFEASIBLE", **body)
                run_for(survivors, 1.0, lambda: s1.epoch != e_bad)
            self.assertGreater(s1.epoch, e_bad, "no re-plan after INFEASIBLE")
            e_new = s1.epoch
            self.assertNotIn(bad_key, [slot_key(p) for p in s1.assign["slots"]])

            # 5. the same (old-epoch) INFEASIBLE again -> no further epoch
            for _ in range(4):
                s1.link.send(0, "INFEASIBLE", **body)
                run_for(survivors, 1.0)
            self.assertEqual(s1.epoch, e_new)
            self.assertEqual(stubs[2].epoch, e_new)
        finally:
            for s in stubs:
                if s.alive:
                    s.kill()
            stop(ground)
            stop(radio)

        # logs
        g = read_jsonl(ground_log)
        assigns = [r for r in g if r["event"] == "assign"]
        self.assertEqual([r["trigger"] for r in assigns], ["start", "lost", "infeasible"])
        self.assertEqual([r["epoch"] for r in assigns], [1, 2, 3])
        self.assertEqual(assigns[1]["changed"], [1, 2, 3])
        self.assertTrue(all("t_wall" in r and "t_mission" in r for r in g))
        live = [(r["agent"], r["from"], r["to"]) for r in g if r["event"] == "liveness"]
        self.assertIn((2, "alive", "suspect"), live)
        self.assertIn((2, "suspect", "lost"), live)
        inf = [r for r in g if r["event"] == "infeasible"]
        self.assertEqual(len(inf), 1)
        self.assertEqual(tuple(inf[0]["slot_key"]), bad_key)
        bl = [r for r in g if r["event"] == "blacklist"]
        self.assertEqual(bl[-1]["entries"][0]["slot_key"], list(bad_key))
        end = [r for r in g if r["event"] == "end"][-1]
        self.assertGreater(end["ignored_old_epoch"], 0)

        stats = [r for r in read_jsonl(radio_log) if r["event"] == "stats"]
        self.assertTrue(stats[-1]["final"])
        tot = stats[-1]["total"]
        radio_loss = tot["dropped_loss"] / tot["sent"]
        print("radio log total: sent=%d delivered=%d dropped_loss=%d (%.3f) no_route=%d "
              "delay mean=%.3f max=%.3f" % (tot["sent"], tot["delivered"], tot["dropped_loss"],
                                            radio_loss, tot["no_route"], tot["delay_mean"],
                                            tot["delay_max"]))
        self.assertTrue(0.15 < radio_loss < 0.25, radio_loss)
        self.assertTrue(0.195 < tot["delay_mean"] < 0.25)


class TestBlackout(unittest.TestCase):
    def test_blackout_both_directions(self):
        pb = free_port_base(10)
        tmp = tempfile.mkdtemp(prefix="swarm_bo_")
        radio_log = os.path.join(tmp, "radio.jsonl")
        radio = spawn(["swarm.radio", "--seed", "1", "--loss", "0.2", "--delay", "0.2",
                       "--port-base", str(pb), "--log", radio_log,
                       "--blackout", "3:1.0:3.0"], radio_log)
        stubs = []
        ground = None
        try:
            time.sleep(0.5)
            ground = RadioLink(0, pb)
            stubs = [Stub(i + 1, pb, config.start_xy(i, 3), rate=10.0) for i in range(3)]
            run_for(stubs, 1.0)                     # registration
            t0 = time.time()
            for _ in range(3):                      # START is lossy too: send a few
                ground.send("all", "START", t0=t0)
            run_for(stubs, 4.5)
        finally:
            for s in stubs:
                s.kill()
            if ground:
                ground.close()
            stop(radio)

        def count(rx, src, lo, hi):
            return sum(1 for m in stubs[rx - 1].inbox
                       if m["type"] == "STATE" and m["src"] == src and lo <= m["t"] - t0 < hi)

        inside = (1.1, 2.9)        # margins for the radio's arrival time vs the sender's "t"
        self.assertEqual(count(3, 1, *inside), 0)      # to 3
        self.assertEqual(count(3, 2, *inside), 0)
        self.assertEqual(count(1, 3, *inside), 0)      # from 3
        self.assertEqual(count(2, 3, *inside), 0)
        self.assertGreater(count(1, 2, *inside), 5)    # others unaffected
        self.assertGreater(count(1, 3, 0.0, 0.9), 0)   # before and after the window
        self.assertGreater(count(1, 3, 3.1, 4.4), 0)
        self.assertGreater(count(3, 1, 3.1, 4.4), 0)
        final = [r for r in read_jsonl(radio_log) if r["event"] == "stats"][-1]
        self.assertGreater(final["pairs"]["1->3"]["dropped_blackout"], 0)
        self.assertGreater(final["pairs"]["3->1"]["dropped_blackout"], 0)
        self.assertEqual(final["pairs"]["1->2"]["dropped_blackout"], 0)


class FakeLink:
    """In-process stand-in for RadioLink (no sockets) for driving Ground.step() directly."""

    def __init__(self):
        self.inbox, self.sent = [], []

    def recv_all(self):
        out, self.inbox = self.inbox, []
        return out

    def send(self, dst, mtype, **body):
        self.sent.append((dst, mtype, body))


class TestGroundRestart(unittest.TestCase):
    def test_restart_resumes_epoch_and_t0(self):
        from swarm.allocator import AllocParams
        from swarm.ground import RESUME_GATHER_S, Ground, JsonlLog
        tmp = tempfile.mkdtemp(prefix="swarm_rs_")
        path = os.path.join(tmp, "ground.jsonl")
        with open(path, "w") as f:      # what the previous ground instance left behind
            f.write(json.dumps({"event": "start", "t0": 1000.0}) + "\n")
            f.write(json.dumps({"event": "assign", "epoch": 7}) + "\n")
        link = FakeLink()
        log = JsonlLog(path)
        g = Ground(3, link, log, AllocParams(), 1e6, SUSPECT_S, LOST_S, 60.0, REPLAN_MIN_S,
                   log_path=path)
        now = 5000.0
        for k in range(int(RESUME_GATHER_S / 0.2) + 2):
            for a in (1, 3):            # agent 2 is gone; 1 and 3 still flying epoch 5
                link.inbox.append({"src": a, "type": "STATE", "pos": [100.0 * a, 50.0, 60.0],
                                   "epoch": 5, "status": "cover"})
            g.step(now + 0.2 * k)
        log.close()
        self.assertEqual(g.phase, "run")
        self.assertEqual(g.t0, 1000.0)
        self.assertEqual(g.epoch, 8)
        self.assertEqual(sorted(g.assignments), [1, 3])
        assigns = [b for d, t, b in link.sent if t == "ASSIGN"]
        self.assertTrue(assigns and all(b["epoch"] == 8 for b in assigns))
        self.assertIn(("all", "START", {"t0": 1000.0}), link.sent)


if __name__ == "__main__":
    unittest.main()
