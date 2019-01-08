#!/usr/bin/env python2

name = "image_saver"

import datetime
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from PIL import Image as i
from sensor_msgs.msg import Image as Imagemsg

class Image(object):
    def __init__(self):
        pass
    
    def Image_save(self, req):
        bridge = CvBridge()
        img_data = bridge.imgmsg_to_cv2(req, 'bgr8')
        cv2.imwrite("/home/amigos/Pictures/capture/{}.jpg".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), img_data)
        print('save picture')
        return

if __name__ == '__main__':
    rospy.init_node(name)
    image =Image()
    sub = rospy.Subscriber('image', Imagemsg, image.Image_save)
    rospy.spin()
