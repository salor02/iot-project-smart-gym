# Romanian Deadlift - LATERAL view required.
# Start position: standing upright (back angle > 150°, legs straight).
# End position:   bent forward at hips (back angle < 120°), legs remain mostly straight.

class RDL:
    def __init__(self):
        self.name = "Romanian Deadlift"

    @staticmethod
    def is_end(angles):
        # Bent forward: back angle < 120° on at least one side,
        # legs stay relatively straight (knee > 140°)
        back_bent = angles["left_back"] < 120 or angles["right_back"] < 120
        legs_straight = angles["left_knee"] > 140 and angles["right_knee"] > 140
        return back_bent and legs_straight

    def is_start(self, angles):
        # Standing upright again
        back_straight = angles["left_back"] > 150 and angles["right_back"] > 150
        legs_straight = angles["left_knee"] > 160 and angles["right_knee"] > 160
        return back_straight and legs_straight
