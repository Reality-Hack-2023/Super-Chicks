# https://clover.coex.tech/en/simple_offboard.html

import rospy
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

from clover.srv import SetLEDEffect

class game_start_end:
    def __init__(self):
        rospy.init_node('game_start_end') 

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)

        # Game initially not started
        self.game_flag = 0
        self.rate = rospy.rate(10)

        rospy.Subscriber('/start_end',
                    Int32, self.start_end)
        
        self.ready_killed_pub = rospy.Publisher('/ready_killed', Int32, queue_size=1)
        self.rate.sleep()
        
        self.set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  

        while not rospy.is_shutdown():
            rospy.spin()

    def start_end(self, data):
        if data.data == 0 and self.game_flag:
            # Game over
            # Go to z = 0
            self.set_effect(r=255, g=0, b=0)
            self.land()
            self.set_effect(r=255, g=255, b=255)
            self.ready_killed_pub.publish(0)
        else:
            # Game started
            # Go to z = 1.5
            self.set_effect(r=0, g=255, b=0)
            self.navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
            self.ready_killed_pub.publish(1)

if __name__ == '__main__':
    try:
        game_start_end()
    except Exception as e:
        print(e)