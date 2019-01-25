#!/usr/bin/env python2

name = "camera_controller"

import rospy
import std_msgs.msg
import time
import os
import sys
sys.path.append("/home/amigos/ros/src/necst_ros3/lib")
import camera

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class cam_controller(object):
    filename = ''

    def __init__(self):
        self.DLSR = camera.controller()
        self.DLSR.detect_camera()
        self.DLSR.set_whitebalance(white='SKY')
        self.DLSR.set_crop(crop='1.3x')

        self.topic_to = rospy.Publisher(
                name = "/camera/image",
                data_class = Image,
                queue_size = 100,
                latch = True,
            )

        topic_from = rospy.Subscriber(
                name = "/camera/oneshot_cmd",
                data_class = std_msgs.msg.String,
                callback = self.callback,
                queue_size = 1,
            )

        pass
    

    def callback(self, req):
        self.filename = req.data + '.jpg'
        if os.path.exists('/home/amigos/Pictures/capture/'+self.filename) == True:
            return

        self.DLSR.shutter_download(filename='/home/amigos/Pictures/capture/'+self.filename)
        print('oneshot!')

        image_path = '/home/amigos/Pictures/capture/'
        img = cv2.imread(image_path + self.filename)
        bridge = CvBridge()
        self.topic_to.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        print('publish picture')
        time.sleep(0.1)
        return

if __name__ == '__main__':
    rospy.init_node(name)
    cam = cam_controller()
    rospy.spin()
