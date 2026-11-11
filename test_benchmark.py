import math
import unittest

from benchmark import clearance_bound, trajectory_errors, wall_distance
from demo import plan


class EvaluationTests(unittest.TestCase):
    def test_relative_error_invariant_under_change_of_world_frame(self):
        truth = [(1, 0, 0), (1, 1, math.pi/2), (0, 1, math.pi)]
        estimate = [(3-y, x+2, angle+math.pi/2) for x,y,angle in truth]
        scores = trajectory_errors(truth, estimate, lag=1)
        self.assertGreater(scores['ate_m'], 1)
        self.assertAlmostEqual(scores['rpe_m'], 0)
        self.assertAlmostEqual(scores['rpe_deg'], 0)

    def test_known_drift_and_wrapped_heading(self):
        truth = [(0,0,0), (1,0,0)]
        est = [(0,0,0), (1.2,0,2*math.pi-.1)]
        scores = trajectory_errors(truth, est, lag=1)
        self.assertAlmostEqual(scores['ate_m'], .2/math.sqrt(2))
        self.assertAlmostEqual(scores['rpe_m'], .2)
        self.assertAlmostEqual(scores['rpe_deg'], math.degrees(.1))

    def test_segment_crossing_obstacle_between_clear_endpoints(self):
        self.assertLessEqual(clearance_bound([(1,3), (5,3)], [(2,2,3,4)]), 0)

    def test_wall_distance_measures_surface(self):
        self.assertEqual(wall_distance(3,3,(1,1,5,5)), 2)
        self.assertEqual(wall_distance(5,3,(1,1,5,5)), 0)

    def test_planner_rejects_unknown_margin(self):
        self.assertEqual(plan({(1,1): -8, (1,2): -8}, (1,1), (1,2)), [])


if __name__ == '__main__':
    unittest.main()
