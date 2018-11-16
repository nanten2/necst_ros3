#! /usr/bin/env python3

name = 'control_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class dome_control_sim(object):
    p = {}
    
    def __init__(self):
        self.pub = rospy.Publisher(
            name = '/necctrl/cpz2724_rsw2/di11',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        pass
    
    def set_local(self):
        self.pub.publish(True)
        return

    def set_remote(self):
        self.pub.publish(False)
        return


if __name__=='__main__':
    rospy.init_node(name)
    c = dome_control_sim()
    c.set_remote()
    rospy.spin()
