#!/usr/bin/env python3

name = "antenna_az"

import rospy
import std_msgs.msg
import topic_utils


def antenna_az_cmd(degree):
    lock = topic_utils.recv(name +"_lock", std_msgs.msg.Bool).data
    
    if lock == False:
        topic_to.publish(degree.data)
    return


if __name__ == "__main__":
    rospy.init_node(name)

    topic_to = rospy.Publisher(
            name = name + "_cmd2",
            data_class = std_msgs.msg.Float64,
            queue_size = 1,
            latch = True
        )

    topic_from = rospy.Subscriber(
            name = name +"_cmd",
            data_class = std_msgs.msg.Float64,
            callback = antenna_az_cmd,
            queue_size = 1,
        )

    rospy.spin()
