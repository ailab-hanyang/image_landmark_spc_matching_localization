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
from std_msgs.msg import String
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

global check
check = True
global cnt
cnt = 0

class ParkingLotSS:
    def __init__(self):
        global check
        #GPU assignment
        os.environ["CUDA_VISIBLE_DEVICES"] = "0"
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

        #Load checkpoint
        self.checkpoint = torch.load(os.path.join("/home/soyeong/MapMatching_66dataset/66_dataset_map_matching_allnew/src/image_to_polygon/data/model_best.pth.tar"))

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
        # self.image_sub = rospy.Subscriber("/image_output", ImageMsg, self.callback, queue_size=1)
        print("check: ", check)
        if check == True:
            self.image_sub = rospy.Subscriber("/cam2/pylon_camera_node/image_raw", ImageMsg, self.callback, queue_size=1, buff_size = 2**24)
        self.image_pub = rospy.Publisher("segmentation_image", ImageMsg, queue_size=1)

    # def callback(self, data):
    #     global check
    #     check = False
    #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     self.test(cv_image)
    #     # try:
    #     #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     # except CvBridgeError as e:
    #     #     print(e)

    #     # self.test(cv_image)
        

    # def test(self, image):
    #     global check
    #     start_time = time.time()

    #     self.model.eval()
    #     torch.set_grad_enabled(False)

    #     tfms = transforms.Compose([
    #         transforms.ToTensor(),
    #         transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    #     ])

    #     inputs = tfms(image).to(self.device)
    #     output = self.model(inputs.unsqueeze(0)).squeeze().cpu().numpy()
    #     pred = np.argmax(output, axis=0)
    #     pred_img = self.label_to_color_image(pred)

    #     msg = self.bridge.cv2_to_imgmsg(pred_img, "bgr8")

    #     inference_time = time.time() - start_time
    #     print("inference time: ", inference_time)

    #     self.image_pub.publish(msg)
    #     check = True
    #     return 0


    def callback(self, data):
        global check
        global cnt
        cnt +=1
        # print(cnt)
        check = False
        if cnt%10 == 0:
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
            check = True
        # else:
            # print("bye~")
        return 0


    def label_to_color_image(self, pred, class_num=4):
        label_colors = np.array([(0, 0, 0), (0, 0, 128), (0, 128, 0), (128, 0, 0)])  #bgr
                                #Unlabeled, Building, Lane-marking, Fence
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
  rate = rospy.Rate(1)

  while not rospy.is_shutdown():
      pred = ParkingLotSS()
      rate.sleep()

if __name__ == '__main__':
    main()

