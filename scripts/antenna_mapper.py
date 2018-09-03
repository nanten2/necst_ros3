#!/usr/bin/env python3

name = "antenna_mapper"

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64


def antenna_mapper_drive(command):
    



if __name__ == "__main__":
    rospy.init_node(name)

    topic_from_drive = rospy.Subscriber(
            name = "antenna_drive_mapper",
            data_class = String,
            callback = antenna_mapper_drive,
            queue_size = 1,
        )

    topic_to_dio = rospy.Publisher(
            name = "cpz2724_rsw1_dio",
            data_class = String,
            queue_size = 1,
            latch = True
        )

    topic_from_az_feedback = rospy.Subscriber(
            name = "feedback_az_speed",
            data_class = Float64,
            callback = antenna_mapper_az,
            queue_size = 1,
        )

    topic_from_el_feedback = rospy.Subscriber(
            name = "feedback_el_speed",
            data_class = Float64,
            callback = antenna_mapper_el,
            queue_size = 1,
        )

    topic_from_mapper = rospy.Subscriber(
            name = name,
            data_class = String,
            callback = antenna_drive_mapper,
            queue_size = 1
        )

    topic_lock = rospy.Subscriber(
            name = name + "_lock",
            data_class = Bool,
            callback = antenna_drive_lock,
            queue_size = 1,
        )


    rospy.spin()
