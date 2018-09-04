#!/usr/bin/env python3

name = "antenna_el_dio"

import pyinterface
import rospy
import std_msgs.msg


board_name = 2724
rsw_id = 0


def antenna_el_do(command):
    if len(command.data) == 16:
        do.output_word("OUT17_32", command.data)
    else:
        print("Command Error")
    return


if __name__ == "__main__":
    rospy.init_node(name)
    do = pyinterface.open(board_name, rsw_id)

    topic_from = rospy.Subscriber(
            name = "cpz2724_rsw0_do17_32",
            data_class = std_msgs.msg.ByteMultiArray,
            callback = antenna_el_do,
            queue_size = 1,
        )

    rospy.spin()
