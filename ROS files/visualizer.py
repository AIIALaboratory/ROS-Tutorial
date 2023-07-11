#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from config_pipe import config as cfg

import os
import sys
import numpy as np
import cv2

class Visualize_node():
    def __init__(self):
        self.class_colors = [[0, 0, 0], [224, 64, 192]]
        self.raw_img = None

        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber('raw_of_mask_image', Image, self.get_image, queue_size = 1)
        self.sub_mask = rospy.Subscriber('mask_image', Image, self.viz_mask, queue_size = 1)

        rospy.loginfo('Vizualization node initiated')

    def get_image(self, img_msg):
        self.raw_img = img_msg

    def viz_mask(self, mask_msg):
        if self.raw_img is not None:
            colors = self.class_colors
            image = self.bridge.imgmsg_to_cv2(self.raw_img, "bgr8")
            pred = self.bridge.imgmsg_to_cv2(mask_msg, "mono8")
            pred = pred.astype('int64')
            comp_img = self.show_prediction(colors, cfg.background, image, pred)

            cv2.namedWindow('comp_image', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('comp_image', 600, 600)
            cv2.imshow('comp_image', comp_img)
            cv2.waitKey(5000)
            cv2.destroyAllWindows()

    def set_img_color(self, colors, background, img, gt, show255=False):
        for i in range(1, len(colors)):
            if i != background:
                img[np.where(gt == i)] = colors[i]
        if show255:
            img[np.where(gt == 255)] = 255
        return img

    def show_prediction(self, colors, background, img, pred):
        im = np.array(img, np.uint8)
        self.set_img_color(colors, background, im, pred)
        final = np.array(im)
        return final


def main(args):
    node_name = 'visualizer'
    rospy.init_node(node_name, anonymous=True)
    Visualize_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
    return 0

if __name__ == '__main__':
    main(sys.argv)