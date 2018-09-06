#!/usr/bin/env python3

name = "antenna_control"

import rospy
import std_msgs.msg

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
        data_class = std_msgs.msg.String,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = name_topic_from, #'cpz2724_rsw0_di26'
        data_class = std_msgs.msg.Bool,
        callback = antenna_control_mapper,
        queue_size = 1,
    )

    rospy.spin()
