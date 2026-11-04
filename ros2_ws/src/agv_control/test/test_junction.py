"""Check junction events against the MTS160 manual's fork and merge figures."""

import unittest

from agv_control.junction import JunctionDetector

ONE = (True, 0, 0, 0, 0)                 # (single_track, left mm, left deg, right mm, right deg)


def feed(messages, detector=None):
    detector = detector or JunctionDetector()
    return [e for m in messages if (e := detector.update(*m)) is not None]


def junction(two_track, n=40):
    return [ONE] * 10 + [two_track] * n + [ONE] * 25


class JunctionTests(unittest.TestCase):

    def test_fork_branch_right(self):        # manual figure 15
        self.assertEqual(feed(junction((False, 0, 0, 5, 30))), ['start', ('end', 'fork')])

    def test_fork_branch_left(self):
        self.assertEqual(feed(junction((False, -5, -30, 0, 0))), ['start', ('end', 'fork')])

    def test_merge_from_right(self):         # manual figure 16
        self.assertEqual(feed(junction((False, 0, 0, 80, -30))), ['start', ('end', 'merge')])

    def test_merge_from_left(self):
        self.assertEqual(feed(junction((False, -80, 30, 0, 0))), ['start', ('end', 'merge')])

    def test_signs_disagree_is_unknown(self):
        self.assertEqual(feed(junction((False, 0, 0, 80, 30))), ['start', ('end', 'unknown')])

    def test_first_gap_is_taken_when_the_track_appears(self):
        # a fork: 5 mm at first, then the tracks spread to 70 mm
        spreading = [(False, 0, 0, 5 + 2 * i, 30) for i in range(35)]
        self.assertEqual(feed([ONE] * 10 + spreading + [ONE] * 25), ['start', ('end', 'fork')])

    def test_start_and_end_timing(self):
        detector = JunctionDetector()
        events = [detector.update(*m) for m in junction((False, 0, 0, 5, 30))]
        self.assertEqual(events.index('start'), 10 + 4)          # 5th two-track message
        self.assertEqual(events.index(('end', 'fork')), 50 + 19)  # 20th single-track message

    def test_short_flicker_is_ignored(self):
        self.assertEqual(feed([ONE] * 10 + [(False, 0, 0, 80, -30)] * 3 + [ONE] * 25), [])

    def test_flicker_does_not_leak_into_the_next_junction(self):
        flicker = [(False, 0, 0, 80, -30)] * 3 + [ONE] * 5   # merge-like noise
        self.assertEqual(feed([ONE] * 10 + flicker + junction((False, 0, 0, 5, 30))),
                         ['start', ('end', 'fork')])

    def test_short_gap_inside_a_junction_is_one_junction(self):
        two = (False, 0, 0, 5, 30)
        messages = [ONE] * 10 + [two] * 20 + [ONE] * 5 + [two] * 20 + [ONE] * 25
        self.assertEqual(feed(messages), ['start', ('end', 'fork')])

    def test_reset_cancels_the_junction(self):
        detector = JunctionDetector()
        events = feed([ONE] * 10 + [(False, 0, 0, 5, 30)] * 20, detector)
        detector.reset()
        events += feed([ONE] * 25, detector)
        self.assertEqual(events, ['start'])


if __name__ == '__main__':
    unittest.main()
