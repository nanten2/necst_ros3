#! /usr/bin/env python3

name = 'dome_door_action_mapper'

# ----
import rospy
import std_msgs.msg

def dome_door_action_mapper(status):
    if status.data == True:
        topic_to.publish('MOVE')
    else:
        topic_to.publish('STOP')
        pass
    return


if __name__=='__main__':
    rospy.init_node(name)
    name_topic_to = rospy.get_param('~name_topic_to')
    name_topic_from = rospy.get_param('~name_topic_from')

    topic_to = rospy.Publisher(
        name = name_topic_to,
        data_class = std_msgs.msg.String,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = name_topic_from,
        data_class = std_msgs.msg.Bool,
        callback = dome_door_action_mapper,
        queue_size = 1,
    )

    rospy.spin()
