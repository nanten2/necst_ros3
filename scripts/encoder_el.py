#!/usr/bin/env python3

name = "encoder_el"

import rospy
import std_msgs.msg


resolution = 360*3600/(23600*400)  #0.13728813559 (4 multiplication)

def encoder_el_mapper(status):
    enc_el = status.data * resolution + 45*3600
    topic_to.publish(enc_el)
    return


if __name__=="__main__":
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = name,
        data_class = std_msgs.msg.Float64,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = name_topic_from,#'cpz6204_rsw0_di2'
        data_class = std_msgs.msg.Int64,
        callback = encoder_el_mapper,
        queue_size = 1,
    )

    rospy.spin()
