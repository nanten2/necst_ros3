#! /usr/bin/env python3

name = 'dome_door_cmd'

# ----
import rospy
import std_msgs.msg

import topic_utils

def dome_door_cmd_mapper(status):
    left_door = topic_utils.recv('dome_door_left_position',
                                 std_msgs.msg.String).data
    right_door = topic_utils.recv('dome_door_right_position',
                                  std_msgs.msg.String).data
    lock = topic_utils.recv('dome_door_lock', std_msgs.msg.Bool).data

    if lock == False:
        if status[0].lower() == 'o':
            # OPEN
            if (left_door == 'OPEN') and (right_door == 'OPEN'):
                pass
            else:
                topic_to1.publish(True)
                topic_to2.publish(True)
                pass
            pass
        
        elif status.data[0].lower() == 'c':
            # CLOSE
            if (left_door == 'CLOSE') and (right_door == 'CLOSE'):
                pass
            else:
                topic_to1.publish(False)
                topic_to2.publish(True)
                pass
            pass

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
