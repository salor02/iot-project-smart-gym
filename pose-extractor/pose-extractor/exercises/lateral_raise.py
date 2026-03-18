# Lateral Raise - FRONTAL view required.
# Start position: arms relaxed at sides (small shoulder angle).
# End position:   arms raised sideways (shoulder angle ~80-100°+)

class LateralRaise:
    def __init__(self):
        self.name = "Lateral Raise"

    @staticmethod
    def is_end(angles):
        # Arms raised: shoulder angle > 70° on both sides
        return (angles["left_shoulder"] > 70 and angles["right_shoulder"] > 70)

    def is_start(self, angles):
        # Arms back down: shoulder angle < 30° on both sides
        return (angles["left_shoulder"] < 30 and angles["right_shoulder"] < 30)
