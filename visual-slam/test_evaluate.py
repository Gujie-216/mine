import unittest

import numpy as np

from evaluate import evaluate, rotations


class ReplayMetrics(unittest.TestCase):
    def setUp(self):
        names = "stamp wall_s cpu_s valid map x y z qx qy qz qw frame_ms".split()
        self.f = np.zeros(61, dtype=[(name, float) for name in names])
        t = np.arange(61)/20
        self.f["stamp"] = self.f["wall_s"] = t
        self.f["cpu_s"] = 1.5*t
        self.f["valid"] = self.f["qw"] = 1
        self.f["frame_ms"] = 42
        self.f["x"], self.f["y"] = np.cos(t), np.sin(t)
        p = np.column_stack([-2*np.sin(t)+5, 2*np.cos(t)-3, t*0])
        q = np.tile([0, 0, 2**-.5, 2**-.5], (len(t), 1))
        self.gt = t, p, rotations(q)

    def test_known_similarity_and_process_load(self):
        r = evaluate(self.f, self.gt, .005)
        self.assertTrue(r["complete"])
        self.assertAlmostEqual(r["scale"], 2)
        self.assertAlmostEqual(r["ate_m"], 0)
        self.assertAlmostEqual(r["rpe_1s_m"], 0)
        self.assertLess(r["rpe_1s_deg"], 1e-5)
        self.assertAlmostEqual(r["process_cpu_pct"], 150)
        self.assertAlmostEqual(r["mean_frame_ms"], 42)
        self.assertEqual(r["rpe_pairs"], 41)

    def test_multiple_maps(self):
        self.f["map"][30:] = 1
        r = evaluate(self.f, self.gt, .005)
        self.assertFalse(r["complete"])
        self.assertIsNone(r["ate_m"])

    def test_missing_associations(self):
        self.f["valid"][20:41] = 0
        r = evaluate(self.f, self.gt, .005)
        self.assertFalse(r["complete"])
        self.assertEqual(r["rpe_pairs"], 0)
        self.assertAlmostEqual(r["coverage"], 40/61)


if __name__ == "__main__":
    unittest.main()
