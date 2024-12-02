# motion_control.py

from geometry_msgs.msg import Twist

class MotionControl(object):
    def __init__(self, cmd_vel_pub):
        self.cmd_vel_pub = cmd_vel_pub

    def move(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        self.move(0.0, 0.0)