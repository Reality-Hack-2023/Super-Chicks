import rospy
from geometry_msgs.msg import Vector3
from interbotix_xs_modules.locobot import InterbotixLocobotCreate3XS as InterbotixLocobotXS
import numpy as np


class ArmControl:
    def __init__(self):

        
        self.locobot = InterbotixLocobotXS(
            robot_model="locobot_wx200", arm_model="mobile_wx200")
        self.rate = rospy.Rate(5)
        self.dead_zone = 5

        rospy.Timer(rospy.Duration(0.2), self.callback)
        rospy.spin()
    
    def callback(self, data):
        data = rospy.wait_for_message("/arm", Vector3)
        if abs(data.x) > self.dead_zone or abs(data.y) > self.dead_zone:
            x = data.x / 45
            y = data.y / 45
            print(f"x: {x} y: {y}")
            self.locobot.camera.pan_tilt_move(x, y)
        else:
            self.locobot.camera.pan_tilt_move(0,0)

if __name__ == '__main__':
    ArmControl()
