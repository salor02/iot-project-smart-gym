# Squat - LATERAL or FRONTAL view.
# Start position: standing upright (back angle > 150°, legs straight).
# End position: legs bent.

class Squat:
    def __init__(self):
        self.name = "Squat"
    
    @staticmethod
    def is_end(angles):
        # Legs bent more than 90°
        return angles["left_knee"] < 90 and angles["right_knee"] < 90
    
    def is_start(self, angles):
        # Standing upright again
        legs_straight  = angles["left_knee"]  > 160 and angles["right_knee"]  > 160
        back_straight  = angles["left_back"]  > 150 and angles["right_back"]  > 150

        return legs_straight and back_straight