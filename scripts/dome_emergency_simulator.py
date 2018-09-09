#! /usr/bin/env python3

name = 'emergency_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class dome_control_sim(object):
    p = {}
    
    def __init__(self, travel_time_left, travel_time_right):        
        self.pub = rospy.Publisher(
            name = '/cpz2724_rsw2/di01',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        pass
    
    def on_emergency(self):
        self.pub.publish(True)
        return

    def off_emergency(self):
        self.pub.publish(False)
        return


if __name__=='__main__':
    rospy.init_node(name)
    c = dome_emergency_sim()
    c.off_emergency()
    rospy.spin()
