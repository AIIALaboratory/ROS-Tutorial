#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
import os
from natsort import natsorted
from cv_bridge import CvBridge


def img_publish():
    rospy.init_node('raw_image_publisher', anonymous = True)
    pub = rospy.Publisher('raw_image', Image, queue_size = 1)

    rate = rospy.Rate(0.2)
    bridge = CvBridge()


    frames_dir = '/home/dimitris/Documents/auth_pipes/Frames/'
    frames = natsorted(os.listdir(frames_dir))
    n = len(frames) - 1
    counter = 0

    while not rospy.is_shutdown():
        raw_img = cv2.imread(os.path.join(frames_dir,frames[counter]))
        pub.publish(bridge.cv2_to_imgmsg(raw_img, "bgr8"))
        counter = counter + 1
        if counter == n:
            counter = 0

        rate.sleep()


if __name__ == '__main__':
    try:
        img_publish()
    except rospy.ROSInterruptException:
        print('Shutting down')
