#!/usr/bin/env python

import sys, cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from ur5_with_robotiq_gripper.msg import Tracker
# from msg import Tracker
from copy import deepcopy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

font = cv2.FONT_HERSHEY_COMPLEX_SMALL

class ObjectRecognition(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("camera1/color/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # We get image dimensions and crop the parts of the image we don't need
        # Bear in mind that because the first value of the image matrix is start and second value is down limit.
        # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
        # To make process faster.
        height, width, channels = cv_image.shape
        # descentre = 160
        # rows_to_watch = 60
        # crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        # crop_img = cv_image[300:height, 300:width]
        # hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([ 0,  219, 0])
        upper_red = np.array([255, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        # kernel = np.ones((5, 5), np.uint8)
        # mask = cv2.erode(mask, kernel)
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        (_, contours, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        sorted_contours = sorted(contours, key=cv2.contourArea, reverse=False)

        #find object
        red_cube1 = sorted_contours[0]

        for cnt in contours:
            area = cv2.contourArea(cnt)
            peri = cv2.arcLength(cnt, True)
            
            if area > 100:
                approx = cv2.approxPolyDP(cnt, 0.01*peri, True)
                
                M = cv2.moments(red_cube1)
                (x,y,w,h)=cv2.boundingRect(approx)

                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # cxy = [str(cx), str(cy)]
                    # rospy.loginfo(cxy)
                    # print"cx: " , str(cx)
                    # print"cy: " , str(cy)

                # if 1 < len(approx) < 10:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,255,0), 2)
                cv2.putText(cv_image, "RedCube", (x-w-40, y-5), font, 0.75, (0, 255, 0), 1)
                cv2.putText(cv_image, "Area: " + str(int(area)), (x-w-40, y-25), font, 0.6, (0, 255, 0), 1)
                cv2.putText(cv_image, ("(CX: " + str(cx) + " CY: " + str(cy) + ")"), (x-w-40, y-40), font, 0.6, (0, 255, 0), 1)
                cv2.drawContours(cv_image, [approx], 0, (255, 255, 255),1)
        
        # result.data = [cx,cy]
        # rospy.loginfo(result)

        cv2.imshow('windows', cv_image)
        # cv2.imshow('hsv', hsv)
        # cv2.imshow('res', res)
        cv2.waitKey(5)
        
        


def main():
    object_recognition = ObjectRecognition()
    rospy.init_node('object_recognition_node', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()