#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('camera_publisher', anonymous=True)
    rate = rospy.Rate(30)  # Adjust the rate as per your requirement (30 FPS in this example)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Error: Failed to open the camera.")
        return

    image_pub = rospy.Publisher('camera_image', Image, queue_size=1)

    while not rospy.is_shutdown():
        success, cv_image = cap.read()

        if not success:
            rospy.logerr("Error: Failed to read the camera frame.")
            break

        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_pub.publish(ros_image)

        cv2.imshow('Camera Feed', cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass