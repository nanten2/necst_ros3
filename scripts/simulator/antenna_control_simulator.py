#! /usr/bin/env python3

name = 'antenna_control_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class antenna_control_sim(object):
    p = {}
    
    def __init__(self):
        self.pub = rospy.Publisher(
            name = '/cpz2724_rsw0/di26',
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
    c = antenna_control_sim()
    c.set_remote()
    rospy.spin()
