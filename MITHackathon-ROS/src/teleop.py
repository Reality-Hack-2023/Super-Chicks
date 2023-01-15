import rospy
from geometry_msgs.msg import Twist

from std_msgs.msg import Float32

from math import pi

# This package is used to control the arm
from interbotix_xs_modules.locobot import InterbotixLocobotCreate3XS as InterbotixLocobotXS

class Teleop:
    def __init__(self):
        rospy.init_node('teleop')

        self.rate = rospy.Rate(60)

        self.lin_x_multiplier = 0.5 # 0.5 m/s
        self.ang_z_multiplier = pi / 2 # pi rad/s

        self.dead_zone = 5 # 5 degree deadzone
        self.max = 35 

        self.normalize = 45

        self.cmd_vel_pub = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=3)
        rospy.Subscriber('/move', Twist, self.control_base)
        rospy.spin()
    def control_base(self, data):
        '''Function to control base, takes x linear and yaw as inputs. 
            Base moves as a velocity in m/s for the duration specified, 
            there is a similair function in https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_xs_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/locobot.py, 
            this function was written to gain a better understanding of how the geometry/Twist message type works.'''


        x = data.linear.x
        yaw = data.angular.z
        msg = Twist()
        msg.angular.y = 0.0
        msg.angular.y = 0.0
        if self.dead_zone < abs(yaw) < self.max:
            msg.angular.z = (yaw / self.normalize) * self.ang_z_multiplier
        elif self.dead_zone >= abs(yaw):
            msg.angular.z = 0
        else:
            if yaw > 0:
                msg.angular.z = self.ang_z_multiplier
            else:
                msg.angular.z = -self.ang_z_multiplier

        if self.dead_zone < abs(x) < self.max:
            msg.linear.x = (x / self.normalize) * self.lin_x_multiplier
        elif self.dead_zone >= abs(x):
            msg.linear.x = 0
        else:
            if x > 0:
                msg.linear.x = self.lin_x_multiplier
            else:
                msg.linear.x = -self.lin_x_multiplier

        msg.linear.y = 0.0
        msg.linear.z = 0.0

        self.cmd_vel_pub.publish(msg)
        self.rate.sleep()

if __name__=='__main__':
    Teleop()
