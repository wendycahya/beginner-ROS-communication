#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

class robot_manipulator:
    def __init__(self):
        # pub_topic_name = "/turtle1/cmd_vel"

        self.pub = rospy.Publisher("Velocity_Feedback", Float32, queue_size=10)
        self.number_publisher = rospy.Subscriber('speedCommand', Float32, self.pose_callback)
        rate = rospy.Rate(10)  # 10hz
        

    def pose_callback(self, data):
        if(data.data >= 807):
            print("Velocity Feedback Fast: ", data.data)
            reading_velocity = data.data
        elif(data.data < 807 and data.data >= 600):
            print("Velocity Feedback Slow Down: ", data.data)
            reading_velocity = data.data
        else:
            print("Velocity Feedback Stop: ", data.data)
            reading_velocity = data.data
        
        self.pub.publish(reading_velocity)

        
if __name__ == '__main__':
    rospy.init_node('robot_receiver', anonymous=True)
    robot_manipulator()
    rospy.spin()