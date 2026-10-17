import math
import unittest
from eye_lab.timing import frame_count


class TimingTests(unittest.TestCase):
    def test_short_positive_runs_have_a_frame_and_cover_the_duration(self):
        for seconds in (.001, .019, .04, .06, .101, 10, 60):
            with self.subTest(seconds=seconds):
                count = frame_count(seconds, .04)
                self.assertGreater(count, 0)
                self.assertLess((count - 1) * .04, seconds)
                self.assertGreaterEqual(count * .04, seconds)

    def test_invalid_durations_fail_before_the_simulation(self):
        for seconds in (0, -1, 61, math.nan, math.inf, True, '10'):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                frame_count(seconds, .04)

    def test_default_benchmark_duration_is_unchanged(self):
        self.assertEqual(frame_count(10, .04), 250)
