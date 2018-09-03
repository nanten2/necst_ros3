#!/usr/bin/env python3

name = 'antenna_emergency'

# ----
import rospy
from std_msgs.msg import Bool

def antenna_emergency_mapper(status):
    topic_to.publish(status.data)
    return


if __name__=='__main__':
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = name,
        data_class = Bool,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = 'cpz2724_rcw0_dio',
        data_class = Bool,
        callback = antenna_emergency_mapper,
        queue_size = 1,
    )

    rospy.spin()
