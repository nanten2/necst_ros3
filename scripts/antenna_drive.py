#!/usr/bin/env python3

name = "antenna_drive"

import rospy
from std_msgs.msg import String


def antenna_drive_mapper(status):
    print(status)
    return

def drive_cmd(command):
    topic_to_mapper.publish(command)
    return


if __name__ == "__main__":
    rospy.init_node(name)

    topic_from = rospy.Subscriber(
            name = name + "_cmd",
            data_class = String,
            callback = drive_cmd,
            queue_size = 1,
        )

    topic_to_mapper = rospy.Publisher(
            name = "cpz2724_rsw1_dio?",
            data_class = String,
            queue_size = 1,
            latch = True
        )

    topic_from_mapper = rospy.Subscriber(
            name = name,
            data_class = String,
            callback = antenna_drive_mapper,
            queue_size = 1
        )

    rospy.spin()
