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
        self.topic_to1 = rospy.Publisher(
            name = 'cpz2724_rsw2_do5',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.topic_to2 = rospy.Publisher(
            name = 'cpz2724_rsw2_do6',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.topic_from1 = rospy.Subscriber(
            name = 'dome_door_left_position',
            data_class = std_msgs.msg.String,
            callback = self.update_left_door,
            queue_size = 1,
        )

        self.topic_from2 = rospy.Subscriber(
            name = 'dome_door_right_position',
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
        if self.left_door == self.right_door:
            topic_to1.publish(False)
            topic_to2.publish(False)
            pass
        return
        


if __name__=='__main__':
    rospy.init_node(name)
    watcher = dome_door_watcher_finalize_move()
    rospy.spin()
