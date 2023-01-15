# https://clover.coex.tech/en/simple_offboard.html

import rospy
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from sensor_msg.msg import Range

class fly:
    def __init__(self):
        rospy.init_node('fly') 

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)

        self.start_end_pub = rospy.Publisher('/start_end', Int32, queue_size=1)

        rospy.Subscriber('/move',
                         Int32, self.linear_z) 

        rospy.Subscriber('/start_end',
                    Int32, self.start_end)
        
        self.rate = rospy.Rate(30)

        self.height = 0
        self.velocity = 0

        rospy.Subscriber('/rangefinder/range', Range, self.set_height)

        while not rospy.is_shutdown():
            self.fly()
            self.rate.sleep()

    def set_height(self, data):

        self.height = data.range

    def start_end(data)

    def fly(self, data):

        if data.data == 1:
            if self.height < 2 and data.data > 1:
                self.navigate(x=0, y=0, z=(data.data + 0.1), speed=0.5, frame_id='body')
        else:
            if data.data  2 and data.data > 1:
                self.navigate(x=0, y=0, z=(data.data + 0.1), speed=0.5, frame_id='body')



if __name__ == '__main__':
    fly()