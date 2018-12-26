#!usr/bin/env python3

name = "camera_simulator"

import rospy
import std_msgs.msg
import time
import os
import sys
sys.path.append("/home/amigos/ros/src/necst_ros3/lib")
import camera

class cam_controller(object):
    filename = ''

    def __init__(self):
        """
        self.topic_to = rospy.Publisher(
                name = "image",
                data_class = Image,
                queue_size = 1,
            )
        """
        topic_from = rospy.Subscriber(
                name = "oneshot_cmd",
                data_class = std_msgs.msg.String,
                callback = self.callback,
                queue_size = 1,
            )

        pass
    

    def callback(self, req):
        self.filename = req.filename + '.jpg'
        print(self.filename)
        print('[CAMERA] ONESHOT!')
        return

    """
    def pub_image(self):
        while True:
            if not self.filename == '':
                break
        while True:
            if os.path.exists('/home/amigos/Pictures/capture/'+self.filename) == True:
                break
            time.sleep(0.1)
        
        image_path = '/home/amigos/Pictures/capture/'
        img = cv2.imread(image_path + self.filename)
        bridge = CvBridge()
        pub = rospy.Publisher('Image', Image, queue_size = 100, latch=True)
        pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        print('publish picture')
        self.filename = ''
        return
    """

if __name__ == '__main__':
    rospy.init_node(name)
    cam = cam_controller()
    rospy.spin()
