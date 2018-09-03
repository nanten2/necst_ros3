#! /usr/bin/env python3

name = 'dome_emergency'

# ----
import rospy
import std_msgs.msg

def dome_emergency_mapper(status):
    topic_to.publish(status)
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
        name = 'cpz2724_rsw2_di1', # dio ch need to be checked
        data_class = std_msgs.msg.Bool,
        callback = dome_emergency_mapper,
        queue_size = 1,
    )

    rospy.spin()
