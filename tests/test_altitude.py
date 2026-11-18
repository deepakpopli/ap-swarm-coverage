"""SWARM_ALT moves the nominal altitude, the band, the safety layers and the lane footprint."""
import json
import os
import subprocess
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PROBE = r"""
import json
from swarm import config, allocator, safety, ground
from swarm.allocator import AllocParams, allocate
p = AllocParams()
a = allocate([1, 2, 3], {1: (140.0, 0.0), 2: (150.0, 0.0), 3: (160.0, 0.0)}, set(), p)
lanes = {i: sorted({s[0] for s in x.slots[x.loop_from:]}) for i, x in a.items()}
ys = sorted({s[1] for s in a[2].slots[a[2].loop_from:]})
print(json.dumps({"alt": config.ALT_NOMINAL, "band": list(config.ALT_BAND),
                  "layers": safety.SafetyParams().layers(),
                  "swath": p.swath_across, "along": p.footprint_along,
                  "ground_swath": ground.AllocParams.swath_across,
                  "lanes": lanes, "ys": [ys[0], ys[-1]]}))
"""


def probe(alt=None):
    env = dict(os.environ, PYTHONPATH=ROOT)
    env.pop("SWARM_ALT", None)
    if alt is not None:
        env["SWARM_ALT"] = str(alt)
    r = subprocess.run([sys.executable, "-c", PROBE], cwd=ROOT, env=env,
                       capture_output=True, text=True)
    if r.returncode:
        raise AssertionError(r.stderr)
    return json.loads(r.stdout)


class AltitudeTest(unittest.TestCase):
    def test_default_unchanged(self):
        d = probe()
        self.assertEqual(d["alt"], 60.0)
        self.assertEqual(d["band"], [50.0, 70.0])
        self.assertEqual(d["layers"], [50.0, 60.0, 70.0])
        self.assertEqual((d["swath"], d["along"], d["ground_swath"]), (102.0, 57.0, 102.0))
        self.assertEqual(d["lanes"]["2"], [150.0])                  # one lane per 100 m strip

    def test_50m(self):
        d = probe(50)
        self.assertEqual(d["alt"], 50.0)
        self.assertEqual(d["band"], [40.0, 60.0])
        self.assertEqual(d["layers"], [40.0, 50.0, 60.0])
        self.assertEqual((d["swath"], d["along"], d["ground_swath"]), (85.0, 48.0, 85.0))
        self.assertEqual(d["lanes"]["2"], [125.0, 175.0])           # two lanes per 100 m strip
        self.assertEqual(d["ys"], [24.0, 276.0])                    # ends inset by 48 / 2

    def test_bad_value(self):
        with self.assertRaises(AssertionError):
            probe(5)


if __name__ == "__main__":
    unittest.main()
