#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
import random

class human_detection:
    def __init__(self):
        #camera connection
        #distance algorithm calculation
        #SSM calculation
        #ROS published
        #pub_topic_name = "/turtle1/cmd_vel"
        #sub_topic_name = "/turtle1/pose"

        self.pub = rospy.Publisher('speedCommand', Int16, queue_size=10)
        self.velocity_feedback = rospy.Subscriber("Velocity_Feedback", Int16, self.pose_callback)
        #self.velocity_msg = Twist()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            num_msg = random.randint(1,10)
            self.pub.publish(num_msg)
            rate.sleep()
            print("Success Sending message, ", num_msg)

    def pose_callback(self, msg):
        print("Alhamdulillah menerima message ", msg)
        
        

        
if __name__ == '__main__':
    rospy.init_node('camera_publisher', anonymous=True)
    human_detection()
    rospy.spin()