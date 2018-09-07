#! /usr/bin/env python3

name = 'dome_door_leftaction_mapper'

# ----
import rospy
import std_msgs.msg

def dome_door_leftaction_mapper(status):
    if status.data == True:
        topic_to.publish('MOVE')
    else:
        topic_to.publish('STOP')
        pass
    return


if __name__=='__main__':
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = 'dome_door_leftdrive',
        data_class = std_msgs.msg.String,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = 'cpz2724_rsw2_di05',
        data_class = std_msgs.msg.Bool,
        callback = dome_door_left_drive_mapper,
        queue_size = 1,
    )

    rospy.spin()
