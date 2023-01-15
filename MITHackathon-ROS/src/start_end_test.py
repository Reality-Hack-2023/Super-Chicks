import rospy
from std_msg.msg import Int32

def start_end_test():
    rospy.init_node('start_end_test')

    start_end_pub = rospy.Publisher('start_end', Int32, queue_size=10)

    rate = rospy.Rate(10) 

    def start():
        user_input = input("Start? ")
        if user_input == " ":
            # Run start sequence
            start_end_pub.publish(1)
    def end():
        user_input = input("End? ")
        if user_input == " ":
            # Run start sequence
            start_end_pub.publish(0)
    
    flag = 0

    while not rospy.is_shutdown():
        if flag == 0:
            start()
        else:
            end()

    rate.sleep()

if __name__ == '__main__':
    try:
        start_end_test()
    except Exception as e:
        print(e)
        

