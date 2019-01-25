#!/usr/bin/env python2

name = "image_saver"

import datetime
import rospy
import std_msgs
import sys
import cv2
from cv_bridge import CvBridge
from PIL import Image as i
from sensor_msgs.msg import Image as Imagemsg

class Image(object):
    filename = ""

    def __init__(self):
        topic_from = rospy.Subscriber(
                name = "oneshot_cmd",
                data_class = std_msgs.msg.String,
                callback = self.callback,
                queue_size = 1,
            )
        pass

    def callback(self, req):
        self.filename = req.data
        return
    
    def Image_save(self, req):
        bridge = CvBridge()
        img_data = bridge.imgmsg_to_cv2(req, 'bgr8')
        cv2.imwrite("/home/amigos/data/opt/{}.jpg".format(self.filename), img_data)
        print('save picture')
        return

if __name__ == '__main__':
    rospy.init_node(name)
    image =Image()
    sub = rospy.Subscriber('image', Imagemsg, image.Image_save)
    rospy.spin()
