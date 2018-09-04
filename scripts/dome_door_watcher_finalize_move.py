#! /usr/bin/env python3

name = 'dome_door_watcher_finalize_move'

# ----
import rospy
import std_msgs.msg

import topic_utils


class dome_door_watcher_finalize_move(object):
    left_door = ''
    right_door = ''
    
    def __init__(self):
        self.topic_to = rospy.Publisher(
            name = 'dome_door_cmd2',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.topic_from1 = rospy.Subscriber(
            name = 'dome_door_leftposition',
            data_class = std_msgs.msg.String,
            callback = self.update_left_door,
            queue_size = 1,
        )

        self.topic_from2 = rospy.Subscriber(
            name = 'dome_door_rightposition',
            data_class = std_msgs.msg.String,
            callback = self.update_right_door,
            queue_size = 1,
        )
        pass

    def update_left_door(self, msg):
        self.left_door = msg.data
        self.check_status()
        return
    
    def update_right_door(self, msg):
        self.right_door = msg.data
        self.check_status()
        return

    def check_status(self):
        if self.left_door == self.right_door != 'MOVING':
            self.topic_to('STAY')
            pass
        return
        


if __name__=='__main__':
    rospy.init_node(name)
    watcher = dome_door_watcher_finalize_move()
    rospy.spin()
