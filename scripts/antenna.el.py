#!/usr/bin/env python3i

name = "antenna_el"

import rospy
from std_msgs.msg import Float64


def antenna_el_cmd(degree):
    topic_to_feedback.publish(degree)
    return

if __name__ == "__main__":
    rospy.init_node(name)

    topic_from = rospy.Subscriber(
            name = name + "_cmd",
            data_class = Float64,
            callback = anntena_el_cmd,
            queue_size = 1,
        )

    topic_to_feedback = rospy.Publisher(
            name = "el_feedback",
            data_class = Float64,
            queue_size = 1,
            latch = True
        )

    rospy.spin()
