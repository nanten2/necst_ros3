#! /usr/bin/env python3

name = "oneshot_achilles"

import sys
import rospy
import time
import std_msgs.msg
sys.path.append("/home/amigos/ros/src/necst_ros3/lib")
import achilles


def callback(req):
    data1 = []
    data2 = []
    array1 = std_msgs.msg.Float64MultiArray()
    array2 = std_msgs.msg.Float64MultiArray()
    
    data = dfs.oneshot(repeat=1, exposure=req.data, starttime=0.0)
    
    [data1.extend(i) for i in list(data[0])]
    [dfs1_list.append(data1[i*16384:(i+1)*16384]) for i in range(int(len(data1)/16384))]
    [data2.extend(i) for i in list(data[1])]
    [dfs2_list.append(data2[i*16384:(i+1)*16384]) for i in range(int(len(data2)/16384))]
    
    array1.data = dfs1_list[0]
    array2.data = dfs2_list[0]
    
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

    dfs = achilles.dfs()

    rospy.spin()
