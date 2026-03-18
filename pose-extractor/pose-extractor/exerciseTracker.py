# idle -> is_standing()? -> start
# start -> is_x_end()? -> exercise x identified, state = end
# end -> is_x_start()? -> rep + 1, state = start
import math
from exercises import Squat, LateralRaise, BicepCurl, RDL

# All supported exercise classes – to add a new exercise just append its class here
EXERCISES = [Squat, LateralRaise, BicepCurl, RDL]

# Returns the angle in degrees at vertex b, formed by segments b->a and b->c
def calculate_angle(a, b, c):
    radians = (
        math.atan2(c["y"] - b["y"], c["x"] - b["x"]) -
        math.atan2(a["y"] - b["y"], a["x"] - b["x"])
    )
    angle = abs(math.degrees(radians))
    if angle > 180.0:
        angle = 360.0 - angle
    return angle

def is_standing(angles):
    legs_straight  = angles["left_knee"]  > 160 and angles["right_knee"]  > 160
    arms_straight  = angles["left_elbow"] > 150 and angles["right_elbow"] > 150
    back_straight  = angles["left_back"]  > 150 and angles["right_back"]  > 150

    return legs_straight and arms_straight and back_straight

class ExerciseTracker:
    def __init__(self):
        self.exercise = None
        self.reps = 0
        self.state = "idle"

    def detect(self, joints):
        angles = dict()
        angles["left_knee"]    = calculate_angle(joints["left_hip"],      joints["left_knee"],  joints["left_ankle"])
        angles["right_knee"]   = calculate_angle(joints["right_hip"],     joints["right_knee"], joints["right_ankle"])
        angles["left_elbow"]   = calculate_angle(joints["left_shoulder"], joints["left_elbow"], joints["left_wrist"])
        angles["right_elbow"]  = calculate_angle(joints["right_shoulder"],joints["right_elbow"],joints["right_wrist"])
        angles["left_back"]    = calculate_angle(joints["left_shoulder"], joints["left_hip"],   joints["left_knee"])
        angles["right_back"]   = calculate_angle(joints["right_shoulder"],joints["right_hip"],  joints["right_knee"])
        angles["left_shoulder"]  = calculate_angle(joints["left_elbow"],  joints["left_shoulder"],  joints["left_hip"])
        angles["right_shoulder"] = calculate_angle(joints["right_elbow"], joints["right_shoulder"], joints["right_hip"])

        if self.state == "idle":
            if is_standing(angles):
                self.state = "start"
            else:
                return
        
        if self.state == "start":
            if self.exercise is None:
                # Try every registered exercise to identify which one is being performed
                for Exercise in EXERCISES:
                    if Exercise.is_end(angles):
                        self.state = "end"
                        self.exercise = Exercise()
                        break
            else:
                if self.exercise.is_end(angles):
                    self.state = "end"
        
        if self.state == "end":
            if self.exercise.is_start(angles):
                self.state = "start"
                self.reps += 1
