"""Junction events from the MTS160 track stream (no ROS)."""


class JunctionDetector:
    '''Turn the MTS160 track stream into junction events'''

    def __init__(self, start=5, end=20, merge_gap=40):
        ''' thresholds,counters in junction and a running sum of angle differences '''

        self.start = start            # two-track messages in a row to enter a junction
        self.end = end                # single-track messages in a row to leave it
        self.merge_gap = merge_gap    # mm; a second track first seen this far away is a merge
        self.reset()

    def reset(self):
        '''Forget any junction in progress (also used after a spin).'''
        self.in_junction = False
        self.wide, self.narrow = 0, 0
        self.angle_sum = 0.0
        self.first_gap = None

    def update(self, single_track, left_pos, left_angle, right_pos, right_angle):
        '''Feed one track message. Returns 'start', ('end', kind) or None.
        kind is 'fork', 'merge' or 'unknown'.'''

        if single_track:
            self.narrow += 1
            self.wide = 0
            if not self.in_junction:          # a short flicker outside a junction: forget it
                self.angle_sum = 0.0
                self.first_gap = None
        else:
            self.wide += 1
            self.narrow = 0
            if self.first_gap is None:        # the moment the second track appeared
                self.first_gap = right_pos - left_pos
            self.angle_sum += right_angle - left_angle

        if not self.in_junction and self.wide >= self.start:
            self.in_junction = True
            return 'start'

        if self.in_junction and self.narrow >= self.end:
            kind = self.classify()
            self.reset()
            return ('end', kind)

        return None

    def classify(self):
        '''Fork: tracks spread apart and start close. Merge: they converge, starting far apart.'''
        if self.angle_sum > 0 and self.first_gap < self.merge_gap:
            return 'fork'
        if self.angle_sum < 0 and self.first_gap >= self.merge_gap:
            return 'merge'
        return 'unknown'
