#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

def num_cb(data):
    number = data.data
    number = number + 10
    print(number)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('int_subscriber', anonymous=True)

    rospy.Subscriber('number_out', Int16, num_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
