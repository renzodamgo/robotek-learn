"""Step 8 - The direction of the lap: which coloured line is longer, confirmed over several frames."""

MIN_LENGTH = 40.0        # px: shorter lines are noise
MIN_CONFIDENCE = 0.25    # how much longer one line must be than the other: |blue - orange| / the longest
CONFIRMATIONS = 5        # consecutive frames that must agree


class DirectionVote:
    """Turns the length of the blue and orange lines seen in each camera frame into a decision.

    Blue longer -> +1 (counter-clockwise lap); orange longer -> -1 (clockwise); unsure -> 0. A frame only counts when
    the longest line is at least ``MIN_LENGTH`` pixels and the lines differ by at least ``MIN_CONFIDENCE``; the
    decision needs ``CONFIRMATIONS`` such frames in a row that name the same direction.
    """

    def __init__(self):
        self.candidate = 0       # what the last frame voted for (0: nothing clear)
        self.hits = 0            # consecutive frames that voted for it
        self.direction = 0

    def update(self, blue_length, orange_length):
        """Feed one frame's two line lengths (pixels). Returns the decision so far: +1, -1 or 0."""
        longest = max(blue_length, orange_length)
        confidence = abs(blue_length - orange_length) / max(longest, 1.0)
        vote = 0
        if longest >= MIN_LENGTH and confidence >= MIN_CONFIDENCE:
            vote = 1 if blue_length > orange_length else -1
        self.hits = self.hits + 1 if vote and vote == self.candidate else (1 if vote else 0)
        self.candidate = vote
        self.direction = vote if self.hits >= CONFIRMATIONS else 0
        return self.direction
