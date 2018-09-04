#!/usr/bin/env python3

name = "antenna_el"

import rospy
import std_msgs.msg


lock = False

def antenna_el_cmd(degree):
    if lock == True:
        return
    else:
        topic_to.publish(degree.data)
    return

def antenna_el_lock(status):
    global lock
    lock = status.data
    return


if __name__ == "__main__":
    rospy.init_node(name)

    topic_to = rospy.Publisher(
            name = name,
            data_class = std_msgs.msg.Float64,
            queue_size = 1,
            latch = True
        )

    topic_from = rospy.Subscriber(
            name = name + "_cmd",
            data_class = std_msgs.msg.Float64,
            callback = antenna_el_cmd,
            queue_size = 1,
        )

    topic_lock = rospy.Subscriber(
            name = name + "_lock",
            data_class = std_msgs.msg.Bool,
            callback = antenna_el_lock,
            queue_size = 1,
        )

    rospy.spin()
