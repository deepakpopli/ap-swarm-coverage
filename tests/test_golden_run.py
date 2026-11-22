"""Golden run: the agents and the ground log exactly what they logged when GOLDEN was recorded.

A deterministic in-process run (3 agents, the ground, the lossy fake radio, stepped time, agent 2
goes silent at 150 s) is hashed file by file, so any change in behaviour shows up as a different
hash. Re-record (python tests/test_golden_run.py) only for a change that is meant to alter it.
"""
import hashlib
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from fakes import FakeRadio, SimClock  # noqa: E402
from sim.fake_vehicle import FakeVehicle  # noqa: E402
from swarm import config  # noqa: E402
from swarm.agent import Agent  # noqa: E402
from swarm.ground import Ground, JsonLines  # noqa: E402

N = 3
TICK_S = 1.0 / config.CONTROL_HZ
RUN_S = 175.0
SILENT_ID, SILENT_AT_S = 2, 150.0     # this agent stops ticking (and so sending) here
FILES = ["agent_1.csv", "agent_2.csv", "agent_3.csv", "agent_1.events.jsonl",
         "agent_2.events.jsonl", "agent_3.events.jsonl", "ground.jsonl", "sent"]
GOLDEN = {
    "agent_1.csv": "4f79b5cec9038108e5fc7c0295e508e9d994eaaf446a13c1fbfde528bab7bbb8",
    "agent_2.csv": "6046f6c15e8b4aa7abb9729963bacacde6bb2aa5277c2fe807864a00a199e7ee",
    "agent_3.csv": "8a0b7416270ed29b3ee8ded60f5d06f3d25c9b12df32447e71168958980a4752",
    "agent_1.events.jsonl": "4bcc6647f9b256fcc705e233268a87e71e88a42aa56d8e31fbdbaa127d4bd1fc",
    "agent_2.events.jsonl": "3d9946af80e282a8264044896f2e9bc77a3818247824503d38539fb431eb4889",
    "agent_3.events.jsonl": "3f30156905f109f01591a5e4313ebf9e9381b0fa991145a0dbc40f83d3839a70",
    "ground.jsonl": "fa8d8265fab60c05bafcbcc8bdf596d4d0ed0ef74efb1e4e30998eb855df3d94",
    "sent": "8460ee00c7d28870b15e3f6234b0f172bb1d1f27be9821fe2ac6543bcafc1cc7",
}


def _agents(radio, clock, out_dir):
    agents = {}
    for a in range(1, N + 1):
        x, y = config.start_xy(a - 1, N)
        vehicle = FakeVehicle(a, (x, y, config.hold_alt(a)), clock=clock)
        agents[a] = Agent(a, vehicle, radio.link(a), str(out_dir))
    return agents


def _tick_agent(agent, now):
    agent.tick(now)
    if agent.takeoff_thread is not None:     # already at altitude: the climb returns at once
        agent.takeoff_thread.join()


def _sent_lines(links):
    """Every message each node sent, in order: type and fields."""
    lines = []
    for node_id, link in sorted(links.items()):
        for msg in link.sent:
            lines.append(f"{node_id} {msg}")
    return "\n".join(lines) + "\n"


def run_scenario(out_dir):
    """Fly the scenario into out_dir; return {file name: sha256}."""
    clock = SimClock()
    radio = FakeRadio(clock, config.RADIO_LOSS, config.RADIO_DELAY_S, seed=7)
    ground_link = radio.link(config.GROUND_ID)
    log = JsonLines(os.path.join(out_dir, "ground.jsonl"))
    ground = Ground(N, ground_link, log, mission_s=1e6)
    agents = _agents(radio, clock, out_dir)
    t_start = clock()
    for _ in range(round(RUN_S / TICK_S)):
        clock.advance(TICK_S)
        now = clock()
        for a, agent in agents.items():
            if not (a == SILENT_ID and now - t_start >= SILENT_AT_S):
                _tick_agent(agent, now)
        ground.step(now)
    for agent in agents.values():
        agent.close()
    log.close()
    links = {a: agent.link for a, agent in agents.items()}
    links[config.GROUND_ID] = ground_link
    with open(os.path.join(out_dir, "sent"), "w") as f:
        f.write(_sent_lines(links))
    digests = {}
    for name in FILES:
        with open(os.path.join(out_dir, name), "rb") as f:
            digests[name] = hashlib.sha256(f.read()).hexdigest()
    return digests


def test_golden_run_is_unchanged(tmp_path):
    assert run_scenario(str(tmp_path)) == GOLDEN


if __name__ == "__main__":
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        for name, digest in run_scenario(tmp).items():
            print(f'    "{name}": "{digest}",')
