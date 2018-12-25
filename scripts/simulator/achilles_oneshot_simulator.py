#! /usr/bin/env python3

name = "achilles_oneshot_simulator"

import sys
import rospy
import std_msgs.msg
sys.path.append("/home/amigos/ros/src/necst_ros3/lib")


def callback(req):
    data1 = []
    data2 = []
    array1 = std_msgs.msg.Float64MultiArray()
    array2 = std_msgs.msg.Float64MultiArray()
    
    array1.data = [[0],[0]]
    array2.data = [[0],[0]]
    
    pub1.publish(array1)
    pub2.publish(array2)
    time.sleep(0.01)
    return

if __name__ == "__main__":
    rospy.init_node(name)

    pub1 = rospy.Publisher(
            name = "/achilles/data1",
            data_class = std_msgs.msg.Float64MultiArray,
            latch = True,
            queue_size = 1,
        )

    pub2 = rospy.Publisher(
            name = "/achilles/data2",
            data_class = std_msgs.msg.Float64MultiArray,
            latch = True,
            queue_size = 1,
        )
    
    sub = rospy.Subscriber(
            name = "/spectrometer/oneshot_cmd",
            data_class = std_msgs.msg.Float32,
            callback = callback,
            queue_size = 1,
        )

    rospy.spin()
