#!/usr/bin/env python3

name = "antenna_el_mapper"

import struct
import rospy
import std_msgs.msg

lock = False

def antenna_el_mapper(command):
    if lock == True:
        topic_to.publish([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        return
    else:
        rate = int(command.data / ((12/7) * (3600/10000)))
        cmd = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', rate)])))
        topic_to.publish(cmd)
    return

def antenna_el_mapper_lock(status):
    global lock
    lock = status.data
    return


if __name__ == "__main__":
    rospy.init_node(name)
    
    topic_to = rospy.Publisher(
            name = "cpz2724_rsw0_do17_32",
            data_class = std_msgs.msg.ByteMultiArray,
            queue_size = 1,
        )

    topic_from = rospy.Subscriber(
            name = "antenna_el_feedback",
            data_class = std_msgs.msg.Float64,
            callback = antenna_el_mapper,
            queue_size = 1,
        )

    topic_lock = rospy.Subscriber(
            name = name + "_lock",
            data_class = std_msgs.msg.Bool,
            callback = antenna_el_mapper_lock,
            queue_size = 1,
        )

    rospy.spin()
