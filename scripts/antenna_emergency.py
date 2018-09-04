#!/usr/bin/env python3

name = 'antenna_emergency'

# ----
import rospy
import std_msgs.msg

def antenna_emergency_mapper(status):
    topic_to.publish(status.data)
    return


if __name__=='__main__':
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = name,
        data_class = std_msgs.msg.Bool,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = 'cpz2724_rcw0_dio',
        data_class = std_msgs.msg.Bool,
        callback = antenna_emergency_mapper,
        queue_size = 1,
    )

    rospy.spin()
