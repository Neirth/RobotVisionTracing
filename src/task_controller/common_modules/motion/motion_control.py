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

    def adjust_direction(self, error, linear_speed=0.2, kp=0.01):
        # Calcular la velocidad angular basada en el error y una constante proporcional
        angular_speed = kp * error
        # Mover el robot con la velocidad lineal y angular calculadas
        self.move(linear=linear_speed, angular=angular_speed)