#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from std_msgs.msg import Float32
import random
import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.PoseModule import PoseDetector

import time as t
import math as mt
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

Achest = 0.04628882081739653
Bchest = -24.603862891449737
Cchest = 3870.586790291231
D= 0
counter = 0
#information
start = datetime.now()
stopwatch_time = t.time()
end_time = datetime.now()
elapsed_time = 0

def main():
    rospy.init_node('camera_publisher', anonymous=True)
    rate = rospy.Rate(30)  # Adjust the rate as per your requirement (30 FPS in this example)
    bridge = CvBridge()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Error: Failed to open the camera.")
        return

    image_pub = rospy.Publisher('camera_image', Image, queue_size=1)

    cap.set(3, 640)  # width
    cap.set(4, 480)  # height
    detector = FaceMeshDetector(maxFaces=1)
    detectorPose = PoseDetector()
    pub = rospy.Publisher('humanDistance', Float32, queue_size=10)
    pubTime = rospy.Publisher('Time', Float32, queue_size=10)
    #velocity_feedback = rospy.Subscriber("Velocity_Feedback", Float32, self.pose_callback)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        success, cv_image = cap.read()

        if not success:
            rospy.logerr("Error: Failed to read the camera frame.")
            break

        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_pub.publish(ros_image)

        height, width, channels = cv_image.shape
        imgMesh, faces = detector.findFaceMesh(cv_image, draw=False)
        #fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)
        cv_image = detectorPose.findPose(cv_image)
        lmList, bboxInfo = detectorPose.findPosition(cv_image, bboxWithHands=False)

        cv2.rectangle(cv_image, (0, 0), (width, 70), (10, 10, 10), -1)
        elapsed_time = round(t.time() - stopwatch_time, 3)

        if bboxInfo:
                idrSh, xrSh, yrSh, zrSh = lmList[11]
                idlSh, xlSh, ylSh, zlSh = lmList[12]
                # print("===== asli =====")
                # print("Id data ", idrSh, "right shoulder x=", xrSh, ", right shoulder y=", yrSh)
                # print("Id data ", idlSh, "left shoulder x=", xrSh, ", left shoulder y=", yrSh)
                chestDistance = round(mt.sqrt((xrSh - xlSh) ** 2 + (yrSh - ylSh) ** 2), 3)

        if faces:
            # skeleton detection
            data = []  # List to store the input data
            while len(data) < 10:
                face = faces[0]
                # print(faces[0])
                pointLeft = face[145]
                pointRight = face[374]
                cv2.line(imgMesh, pointLeft, pointRight, (0, 200, 0), 3)
                cv2.circle(imgMesh, pointLeft, 5, (255, 0, 255), cv2.FILLED)
                cv2.circle(imgMesh, pointRight, 5, (255, 0, 255), cv2.FILLED)
                w, _ = detector.findDistance(pointLeft, pointRight)
                W = 6.3  # default real width eyes distance
                f = 714  # finding the average for focal length
                d = ((W * f) / w) * 10
                d = round(d, 3)

                real_measurement = round(
                    (Achest * (chestDistance ** 2)) + (Bchest * chestDistance) + Cchest, 2)
                D = min(d, real_measurement)

                data.append(D)

            # Calculate the average
            D = sum(data) / len(data)
            D = round(D, 3)

        cvzone.putTextRect(cv_image, f'Depth: {D} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)
        pubTime.publish(elapsed_time)
        pub.publish(D)
        rate.sleep()
        print("Success Sending message, ", D, " on time: ", elapsed_time)
            
        cv2.putText(cv_image, "{} s".format(elapsed_time), (10, 30), cv2.FONT_HERSHEY_PLAIN,
                    2, (15, 225, 215), 2)
        cv2.putText(cv_image, "counter {}".format(counter), (10, 60), cv2.FONT_HERSHEY_PLAIN,
                    2, (15, 225, 215), 2)
        # # Display the video frame in the 'Video Stream' window
        cv2.imshow('Camera Feed', cv_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass