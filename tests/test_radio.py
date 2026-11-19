"""Radio emulator: drop decisions and windows without sockets, plus one slow end-to-end test."""
import json
import os
import subprocess
import sys
import time

import pytest

from sim.radio import drop_draw, window_drop
from swarm.net import RadioLink

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_drop_decision_is_a_function_of_seed_src_dst_seq():
    assert drop_draw(7, 1, 2, 42) == drop_draw(7, 1, 2, 42)
    assert len({drop_draw(7, 1, 2, seq) for seq in range(100)}) == 100
    assert drop_draw(7, 1, 2, 42) != drop_draw(8, 1, 2, 42)
    assert drop_draw(7, 1, 2, 42) != drop_draw(7, 2, 1, 42)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_measured_loss_rate(seed):
    draws = [drop_draw(seed, src, dst, seq) for src, dst in [(1, 2), (2, 0), (0, 3)]
             for seq in range(1, 1668)]                       # about 5000 copies
    loss = sum(u < 0.2 for u in draws) / len(draws)
    assert loss == pytest.approx(0.2, abs=0.02)               # 3.5 standard deviations


def test_blackout_cuts_both_directions():
    blackouts = [(3, 10.0, 20.0)]
    assert window_drop(3, 1, 10.0, blackouts, [])             # from 3
    assert window_drop(0, 3, 19.9, blackouts, [])             # to 3
    assert not window_drop(1, 2, 15.0, blackouts, [])         # others unaffected
    assert not window_drop(3, 1, 9.9, blackouts, [])          # [T0, T1)
    assert not window_drop(1, 3, 20.0, blackouts, [])


def test_mute_cuts_only_what_the_agent_sends():
    mutes = [(2, 5.0, 8.0)]
    assert window_drop(2, 1, 6.0, [], mutes)
    assert window_drop(2, 0, 6.0, [], mutes)
    assert not window_drop(1, 2, 6.0, [], mutes)              # it still hears
    assert not window_drop(2, 1, 8.0, [], mutes)


def start_radio(port_base, log_path):
    proc = subprocess.Popen([sys.executable, "-m", "sim.radio", "--seed", "1", "--loss", "0",
                             "--delay", "0.2", "--port-base", str(port_base), "--log", log_path],
                            cwd=REPO)
    time.sleep(0.5)
    return proc


def exchange(sender, receiver, count):
    """Send `count` messages, one per 10 ms poll, and return those that arrived."""
    got = []
    sent = 0
    end = time.time() + 3.0
    while time.time() < end and len(got) < count:
        if sent < count:
            sender.send(receiver.node_id, "PING")
            sent += 1
        got += [m for m in receiver.recv_all() if m["type"] == "PING"]
        time.sleep(0.01)
    return got


@pytest.mark.slow
def test_delay_and_reregistration_after_radio_restart(tmp_path):
    port_base = 20000 + os.getpid() % 500 * 10
    log_path = str(tmp_path / "radio.jsonl")
    a, b = RadioLink(1, port_base), RadioLink(2, port_base)
    radio = start_radio(port_base, log_path)
    try:
        a.recv_all(), b.recv_all()                            # HELLO
        a.next_hello = b.next_hello = 0.0
        got = exchange(a, b, 20)
        delays = [m["t_recv"] - m["t"] for m in got]
        assert len(got) == 20
        assert 0.195 <= min(delays) and max(delays) < 0.25
        radio.terminate()
        radio.wait(5)
        radio = start_radio(port_base, log_path)              # forgets every node
        time.sleep(1.1)                                       # one HELLO period
        assert len(exchange(a, b, 10)) == 10
    finally:
        radio.terminate()
        radio.wait(5)
        a.close()
        b.close()
    with open(log_path) as f:
        stats = [r for r in map(json.loads, f) if r["event"] == "stats"]
    assert [s["pairs"]["1->2"]["delivered"] for s in stats] == [20, 10]
