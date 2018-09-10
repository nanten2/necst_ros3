#!/usr/bin/env python3

name = "antenna_az_mapper"

import struct
import rospy
import std_msgs.msg
import topic_utils

def antenna_az_mapper(command):
    lock = topic_utils.recv(name +"_lock", std_msgs.msg.Bool).data
    
    if lock == True:
        cmd = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    else:
        rate = int(command.data / ((12/7) * (3600/10000)))
        cmd = list(map(int,  ''.join([format(b, '08b')[::-1] for b in struct.pack('<h', rate)])))
    
    for i in range(0,16):
        topic_to[i].publish(cmd[i])
        continue

    return


if __name__ == "__main__":
    rospy.init_node(name)
    
    topic_to = []
    for i in range(1,17):
        topic_to_ = rospy.Publisher(
            name = "cpz2724_rsw0_do%d"%(i),
            data_class = std_msgs.msg.Byte,
            queue_size = 1,
        )
        topic_to.append(topic_to_)
        continue

    topic_from = rospy.Subscriber(
            name = "antenna_az_feedback",
            data_class = std_msgs.msg.Float64,
            callback = antenna_az_mapper,
            queue_size = 1,
        )

    rospy.spin()
