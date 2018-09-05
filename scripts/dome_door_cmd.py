#! /usr/bin/env python3

name = 'dome_door_cmd'

# ----
import rospy
import std_msgs.msg

import topic_utils

def dome_door_cmd_handler(msg):
    left_door = topic_utils.recv('dome_door_leftposition',
                                 std_msgs.msg.String).data
    right_door = topic_utils.recv('dome_door_rightposition',
                                  std_msgs.msg.String).data
    lock = topic_utils.recv('dome_door_lock', std_msgs.msg.Bool).data

    if lock == False:
        if msg.data[0].lower() == 'o':
            # OPEN
            if (left_door == 'OPEN') and (right_door == 'OPEN'):
                cmd2 = 'STAY'
            else:
                cmd2 = 'OPEN'
                pass
        
        elif msg.data[0].lower() == 'c':
            # CLOSE
            if (left_door == 'CLOSE') and (right_door == 'CLOSE'):
                cmd2 = 'STAY'
            else:
                cmd2 = 'CLOSE'
                pass
            
        elif msg.data[0].lower() == 's':
            # STOP
            cmd2 = 'STAY'
            pass

        topic_to.publish(cmd2)
        pass
    return


if __name__=='__main__':
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = 'dome_door_cmd2',
        data_class = std_msgs.msg.String,
        queue_size = 1,
    )

    topic_from = rospy.Subscriber(
        name = name,
        data_class = std_msgs.msg.String,
        callback = dome_door_cmd_handler,
        queue_size = 1,
    )

    rospy.spin()
