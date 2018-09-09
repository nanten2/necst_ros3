#!/usr/bin/env python3

name = "spectrometer_data2"

import rospy
import std_msgs.msg

def spectrometer_data2_mapper(status):
    if status.data:
        topic_to.publish(status.data)
    else:
        pass
    return


if __name__=="__main__":
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = name,
        data_class = std_msgs.msg.Float64MultiArray,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = "XFFTS_SPEC2",
        data_class = std_msgs.msg.Float64MultiArray,
        callback = spectrometer_data2_mapper,
        queue_size = 1,
    )

    rospy.spin()
