#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PIL import Image
import numpy as np
import os
import sys
import cv2
import time

import torch
import torchvision.transforms as transforms
import torchvision

from modeling.deeplab import DeepLab

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge.boost.cv_bridge_boost import getCvType
from sensor_msgs.msg import Image as ImageMsg
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

class DeeplabRos:
    def __init__(self):

        #GPU assignment
        os.environ["CUDA_VISIBLE_DEVICES"] = "0"
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

        #Load checkpoint
        self.checkpoint = torch.load(os.path.join("./src/deeplab_ros/data/model_best.pth.tar"))

        #Load Model
        self.model = DeepLab(num_classes=4,
                    backbone='mobilenet',
                    output_stride=16,
                    sync_bn=True,
                    freeze_bn=False)

        self.model.load_state_dict(self.checkpoint['state_dict'])
        self.model = self.model.to(self.device)

        #ROS init
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cam2/pylon_camera_node/image_raw", ImageMsg, self.callback, queue_size=1, buff_size = 2**24)
        self.image_pub = rospy.Publisher("segmentation_image", ImageMsg, queue_size=1)
        

    def callback(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        start_time = time.time()

        self.model.eval()
        torch.set_grad_enabled(False)

        tfms = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

        inputs = tfms(cv_image).to(self.device)
        output = self.model(inputs.unsqueeze(0)).squeeze().cpu().numpy()
        pred = np.argmax(output, axis=0)
        pred_img = self.label_to_color_image(pred)

        msg = self.bridge.cv2_to_imgmsg(pred_img, "bgr8")

        inference_time = time.time() - start_time
        print("inference time: ", inference_time)

        self.image_pub.publish(msg)


    def label_to_color_image(self, pred, class_num=4):
        label_colors = np.array([(0, 0, 0), (0, 0, 128), (0, 128, 0), (128, 0, 0)])  #bgr
                                # Unlabeled, Building, Lane-marking, Fence
        r = np.zeros_like(pred).astype(np.uint8)
        g = np.zeros_like(pred).astype(np.uint8)
        b = np.zeros_like(pred).astype(np.uint8)

        for i in range(0, class_num):
            idx = pred == i
            r[idx] = label_colors[i, 0]
            g[idx] = label_colors[i, 1]
            b[idx] = label_colors[i, 2]
        
        rgb = np.stack([r, g, b], axis=2)

        return rgb

def main():
  rospy.init_node('inference', anonymous=True)
  pred = DeeplabRos()
  rate = rospy.Rate(8)

  while not rospy.is_shutdown():
      rospy.spin()
      rate.sleep()

if __name__ == '__main__':
    main()

