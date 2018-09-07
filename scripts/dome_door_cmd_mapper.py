#! /usr/bin/env python3

name = 'dome_door_cmd_mapper'

# ----
import rospy
import std_msgs.msg

import topic_utils

def dome_door_cmd_mapper(msg):
    lock = topic_utils.recv('dome_door_lock', std_msgs.msg.Bool).data

    if lock == False:
        if status[0].lower() == 'o':
            # OPEN
            topic_to1.publish(True)
            topic_to2.publish(True)
            
        elif status.data[0].lower() == 'c':
            # CLOSE
            topic_to1.publish(False)
            topic_to2.publish(True)
            pass

        elif status.data[0].lower() == 's':
            # STOP
            topic_to1.publish(False)
            topic_to2.publish(False)
            pass
        pass
    return


if __name__=='__main__':
    rospy.init_node(name)

    topic_to1 = rospy.Publisher(
        name = 'cpz2724_rsw2_do05',
        data_class = std_msgs.msg.Bool,
        queue_size = 1,
    )

    topic_to2 = rospy.Publisher(
        name = 'cpz2724_rsw2_do06',
        data_class = std_msgs.msg.Bool,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = 'dome_door_cmd2',
        data_class = std_msgs.msg.String,
        callback = dome_door_cmd_mapper,
        queue_size = 1,
    )

    rospy.spin()
