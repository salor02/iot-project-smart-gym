# Bicep Curl - FRONTAL view required.
# Start position: arms extended (elbow angle > 150°).
# End position:   arms curled (elbow angle < 60°), upper arms stay at sides.

class BicepCurl:
    def __init__(self):
        self.name = "Bicep Curl"

    @staticmethod
    def is_end(angles):
        # Arms curled: elbow angle < 60° and arms stay close to body (shoulder < 40°)
        elbows_bent = angles["left_elbow"] < 60 and angles["right_elbow"] < 60
        arms_at_sides = angles["left_shoulder"] < 40 and angles["right_shoulder"] < 40
        return elbows_bent and arms_at_sides

    def is_start(self, angles):
        # Arms extended back down: elbow angle > 150°
        return (angles["left_elbow"] > 150 and angles["right_elbow"] > 150)
