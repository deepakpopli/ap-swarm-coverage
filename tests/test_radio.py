"""Radio emulator: routing, seeded loss and windows in-process, plus delay over real sockets."""
import json
import os
import subprocess
import sys
import time

import pytest

from sim.radio import Radio, window_drop
from swarm import config, protocol
from swarm import swarm_mav as mav
from swarm.net import RadioLink

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GROUND = config.GROUND_ID
STATE = protocol.state(0.0, (1.0, 2.0, 50.0), (0.0, 0.0, 0.0), 1, 0, "cover")


def port_base(k):
    """A port range per test and per pytest process, clear of a running mission's."""
    return 20000 + os.getpid() % 400 * 20 + k


def packet(src, msg) -> bytes:
    return bytes(msg.pack(mav.MAVLink(None, srcSystem=src, srcComponent=protocol.COMPONENT)))


def radio(tmp_path, k, seed=1, loss=0.2, blackouts=()):
    return Radio(3, seed, loss, 0.2, port_base(k), str(tmp_path / f"radio{k}.jsonl"),
                 list(blackouts))


def queued(r):
    return sorted((src, dst) for _, _, _, src, dst, _ in r.queue)


def test_broadcast_goes_to_every_node_but_the_sender(tmp_path):
    r = radio(tmp_path, 0, loss=0.0)
    r.on_datagram(packet(2, STATE), 100.0)
    assert queued(r) == [(2, 1), (2, 3), (2, GROUND)]


def test_assign_and_infeasible_go_to_target_system_only(tmp_path):
    r = radio(tmp_path, 1, loss=0.0)
    r.on_datagram(packet(GROUND, protocol.assign(3, 5, [0, 1, 2, 3], [(1.0, 2.0)], 0)), 100.0)
    r.on_datagram(packet(1, protocol.infeasible(5, 0, (1.0, 2.0))), 100.0)
    assert queued(r) == [(1, GROUND), (GROUND, 3)]


def drops(r, src, dst, count):
    """Route `count` copies src -> dst; True where the copy was dropped for loss."""
    out = []
    for _ in range(count):
        stats = r.stats.get(f"{src}->{dst}")
        lost_before = stats.dropped_loss if stats else 0
        r.route(b"", src, dst, 100.0)
        out.append(r.stats[f"{src}->{dst}"].dropped_loss > lost_before)
    r.queue.clear()
    return out


def test_seeded_loss_is_reproducible_and_about_20_percent(tmp_path):
    a, b, c = radio(tmp_path, 2), radio(tmp_path, 3), radio(tmp_path, 4, seed=2)
    for src, dst in [(1, 2), (2, 1), (GROUND, 3), (3, GROUND)]:
        pattern = drops(a, src, dst, 10000)
        assert drops(b, src, dst, 10000) == pattern          # same seed: same copies dropped
        assert drops(c, src, dst, 10000) != pattern          # another seed: others
        assert sum(pattern) / 10000 == pytest.approx(0.2, abs=0.014)   # 3.5 sigma
        assert pattern[:256] != pattern[256:512]             # no 256-message period


def test_window_drops_need_t0_from_start(tmp_path):
    r = radio(tmp_path, 5, loss=0.0, blackouts=[(3, 10.0, 20.0)])
    r.on_datagram(packet(3, STATE), 1015.0)                  # no t0 yet: no window
    assert len(r.queue) == 3
    r.on_datagram(packet(GROUND, protocol.start(1000.0)), 1001.0)
    assert r.t0 == 1000.0
    r.queue.clear()
    r.on_datagram(packet(3, STATE), 1015.0)                  # blackout: from 3 ...
    r.on_datagram(packet(1, STATE), 1015.0)                  # ... and to 3
    assert queued(r) == [(1, 2), (1, GROUND)]
    assert r.stats["1->3"].dropped_window == 1 and r.stats["3->1"].dropped_window == 1


def test_blackout_cuts_both_directions():
    blackouts = [(3, 10.0, 20.0)]
    assert window_drop(3, 1, 10.0, blackouts)                 # from 3
    assert window_drop(GROUND, 3, 19.9, blackouts)            # to 3
    assert not window_drop(1, 2, 15.0, blackouts)             # others unaffected
    assert not window_drop(3, 1, 9.9, blackouts)              # [T0, T1)
    assert not window_drop(1, 3, 20.0, blackouts)


def test_delay_over_sockets(tmp_path):
    """Real relay, no loss: 20 STATEs from 1 reach 2 after about 0.2 s; agent 3 is not running."""
    base = port_base(6)
    log_path = str(tmp_path / "radio.jsonl")
    a, b = RadioLink(1, base), RadioLink(2, base)
    proc = subprocess.Popen([sys.executable, "-m", "sim.radio", "--n", "3", "--seed", "1",
                             "--loss", "0", "--delay", "0.2", "--port-base", str(base),
                             "--log", log_path], cwd=REPO)
    got, sent_at = [], []
    try:
        time.sleep(0.5)
        end = time.time() + 3.0
        while time.time() < end and len(got) < 20:
            if len(sent_at) < 20:      # the epoch field numbers the messages
                sent_at.append(time.time())
                a.send(protocol.state(sent_at[-1], (1.0, 2.0, 50.0), (0.0, 0.0, 0.0),
                                      len(sent_at) - 1, 0, "cover"))
            got += b.recv_all()
            time.sleep(0.01)
    finally:
        proc.terminate()
        proc.wait(5)
        a.close()
        b.close()
    assert len(got) == 20 and all(m.get_srcSystem() == 1 for m in got)
    delays = [m.t_recv - sent_at[m.epoch] for m in got]
    assert 0.195 <= min(delays) and max(delays) < 0.25
    with open(log_path) as f:
        [stats] = [r for r in map(json.loads, f) if r["event"] == "stats"]
    pair = stats["pairs"]["1->2"]
    assert pair["delivered"] == 20 and pair["delay_mean_s"] == pytest.approx(0.2, abs=0.01)
    assert stats["pairs"]["1->3"]["delivered"] == 20          # nobody listens: harmless
