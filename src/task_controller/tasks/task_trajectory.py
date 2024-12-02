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

                rospy.loginfo("El robot ha avanzado 800 mm")

        elif self.stage == 1:
            # Girar 90 grados
            self.motion_control.move(angular=0.5)
            if current_time >= 1.57:
                self.motion_control.stop()
                self.start_time = time()
                self.stage += 1

                rospy.loginfo("El robot ha girado 90 grados")
        
        elif self.stage == 2:
            self.motion_control.stop()
            self.stage += 1
            rospy.loginfo("El robot se ha parado")
    
        else:
            rospy.signal_shutdown("El robot ha completado la trayectoria, se puede detener el nodo.")
