# task_trajectory.py

import rospy
from time import time

class TaskTrajectory:
    def __init__(self, motion_control):
        self.motion_control = motion_control
        self.start_time = time()
        self.stage = 0

    def execute(self):
        current_time = time() - self.start_time

        if self.stage == 0:
            # Mover hacia adelante 800 mm
            self.motion_control.move(linear=0.2)
            if current_time >= 4.0:
                self.motion_control.stop()
                self.start_time = time()
                self.stage += 1

        elif self.stage == 1:
            # Girar 90 grados
            self.motion_control.move(angular=0.5)
            if current_time >= 1.57:
                self.motion_control.stop()
                self.start_time = time()
                self.stage += 1

        else:
            self.motion_control.stop()
            rospy.loginfo("Trayectoria completada")