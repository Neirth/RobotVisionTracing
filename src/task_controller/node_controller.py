#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# Importar módulos desde tu estructura de proyecto
import os


# Importar módulos comunes y tareas
from common_modules.motion.motion_control import MotionControl
# from common_modules.sensors.lidar_processing import LidarProcessing
# from common_modules.sensors.camera_processing import CameraProcessing
from tasks.task_trajectory import TaskTrajectory
# from tasks.task_line_following import TaskLineFollowing
# from tasks.task_navigation import TaskNavigation
# from tasks.task_beacon_positioning import TaskBeaconPositioning
# from tasks.task_object_handling import TaskObjectHandling
# from tasks.task_color_sorting import TaskColorSorting

CURRENT_TASK = 'trajectory'

class NodeController:
    def __init__(self):
        rospy.init_node('node_controller', anonymous=False)

        # Publicadores y Suscriptores
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_callback)

        # Utilidades
        self.bridge = CvBridge()
        self.motion_control = MotionControl(self.cmd_vel_pub)
        # self.lidar_processing = LidarProcessing()
        # self.camera_processing = CameraProcessing()

        # Tareas
        self.tasks = {
            'trajectory': TaskTrajectory(self.motion_control),
            # 'line_following': TaskLineFollowing(self.motion_control, self.camera_processing),
            # 'navigation': TaskNavigation(self.motion_control, self.lidar_processing),
            # 'beacon_positioning': TaskBeaconPositioning(self.motion_control, self.lidar_processing),
            # 'object_handling': TaskObjectHandling(self.motion_control, self.camera_processing),
            # 'color_sorting': TaskColorSorting(self.motion_control, self.camera_processing)
        }

        # Estado
        self.current_task = CURRENT_TASK
        self.rate = rospy.Rate(10)  # Frecuencia de ejecución en Hz

    def run(self):
        while not rospy.is_shutdown():
            if self.current_task in self.tasks:
                self.tasks[self.current_task].execute()
            else:
                rospy.logwarn("Tarea desconocida: {}".format(self.current_task))
            self.rate.sleep()

    def lidar_callback(self, data):
        # Procesar datos del LIDAR si es necesario
        pass

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Procesar imagen de la cámara si es necesario
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        node = NodeController()
        node.run()
    except rospy.ROSInterruptException:
        pass