#! /usr/bin/env python3

name = 'dome_door_cmd'

# ----
import rospy
import std_msgs.msg

def dome_door_cmd_mapper(status):
    left_door = 
    
    if status.data[0].lower() == 'o':
        # OPEN
        topic_to.publish('LOCAL')
    else:
        topic_to.publish('REMOTE')
        pass
    return


if __name__=='__main__':
    rospy.init_node(name)

    topic_to1 = rospy.Publisher(
        name = 'cpz2724_rsw2_do5',
        data_class = std_msgs.msg.Bool,
        queue_size = 1,
    )

    topic_to2 = rospy.Publisher(
        name = 'cpz2724_rsw2_do6',
        data_class = std_msgs.msg.Bool,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = name,
        data_class = std_msgs.msg.String,
        callback = dome_door_cmd_mapper,
        queue_size = 1,
    )

    rospy.spin()
