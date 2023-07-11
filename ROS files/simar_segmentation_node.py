#!/usr/bin/env python3

import rospy, rospkg, roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from config_pipe import config as cfg
from network import PipeSeg
from predict import predict

import torch
import torchvision
import torch.nn.functional as F
import torch.backends.cudnn as cudnn

import os
import sys
import numpy as np
import cv2

class Pipeline_Segmentation_node():
    def __init__(self):

        # CUDA settings
        self.device = ("cuda" if torch.cuda.is_available() else "cpu")

        # set paths
        #self.rospack = rospkg.RosPack()
        #self.base_path = self.rospack.get_path("visualanalysis_acw")
        #self.dir_path = os.path.dirname(os.path.realpath(__file__))
        #common_path = self.rospack.get_path("ac_tools")

        #self.model_segmentation_weights_path = rospy.get_param("model_weights_path", "data/models/SIMAR_Segmentation_Weights/model.pth")
        #self.model_weights_path = os.path.join(self.base_path, self.model_segmentation_weights_path)

        self.model_weights_path = '/home/dimitris/ros_seg/src/robot_pkg/src/log/pipe_detection/snapshot/epoch-86.pth'

        #if not os.path.exists(self.model_weights_path):
        #    command = os.path.join(common_path, 'scripts/gdrive.sh') + ' ' + rospy.get_param("gdrive_url_seg") + ' ' + self.model_weights_path
        #    dc = call(command, shell=True)

        # init model
        self.model_segmentation = PipeSeg(cfg.num_classes, is_training = False, criterion = None, ohem_criterion = False)
        state_dict = torch.load(self.model_weights_path, map_location = torch.device('cpu'))
        if 'model' in state_dict.keys():
            state_dict = state_dict['model']
        self.model_segmentation.load_state_dict(state_dict, strict = False)
        self.model_segmentation.to(self.device)

        self.bridge = CvBridge()
        #self.img_sub_topic = rospy.get_param('~image_topic', 'raw_image')
        self.img_sub_topic = 'raw_image'
        self.sub_image = rospy.Subscriber( self.img_sub_topic, Image, self.semantic_segmentation, queue_size = 1)
        #self.mask_pub_topic = rospy.get_param('~segmentation_mask_topic')
        self.mask_pub_topic = 'mask_image'
        self.pub_mask = rospy.Publisher(self.mask_pub_topic, Image, queue_size = 1)
        #self.img_pub_topic = rospy.get_param('~segmentation_image_topic')
        self.img_pub_topic = 'raw_of_mask_image'
        self.pub_image = rospy.Publisher(self.img_pub_topic, Image, queue_size = 1)

        rospy.loginfo('Pipeline Segmentation node initiated')

    def semantic_segmentation(self, img_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        mask, frame_processed = predict(cfg, self.model_segmentation, frame, self.device)
        mask = mask.astype('uint8')
        
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame_processed, "bgr8"))
        self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))


def main(args):
    node_name = 'simar_segmentation_node'
    rospy.init_node(node_name, anonymous=True)
    Pipeline_Segmentation_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
    return 0

if __name__ == '__main__':
    main(sys.argv)