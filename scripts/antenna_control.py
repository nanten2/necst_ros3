#!/usr/bin/env python3

name = "antenna_control"

# ----
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String

def antenna_control_mapper(status):
    if status.data == True:
        topic_to.publish("LOCAL")
    else:
        topic_to.publish("REMOTE")
        pass
    return


if __name__=="__main__":
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = name,
        data_class = String,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = 'cpz2724_rcw0_dio',
        data_class = Bool,
        callback = antenna_control_mapper,
        queue_size = 1,
    )

    rospy.spin()
