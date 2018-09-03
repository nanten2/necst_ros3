#!/usr/bin/env python3

name = "antenna_el"

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool


lock = False

def antenna_el_cmd(degree):
    if lock == True:
        return
    else:
        topic_to_feedback.publish(degree.data)
    return

def antenna_el_lock(status):
    global lock
    lock = status.data
    return


if __name__ == "__main__":
    rospy.init_node(name)

    topic_from = rospy.Subscriber(
            name = name + "_cmd",
            data_class = Float64,
            callback = antenna_el_cmd,
            queue_size = 1,
        )

    topic_to_feedback = rospy.Publisher(
            name = "el_feedback",
            data_class = Float64,
            queue_size = 1,
            latch = True
        )

    topic_lock = rospy.Subscriber(
            name = name + "_lock",
            data_class = Bool,
            callback = antenna_el_lock,
            queue_size = 1,
        )

    rospy.spin()
