#! /usr/bin/env python3

name = 'dome_error_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg



class dome_error_sim(object):
    p = {}
    
    def __init__(self):
        self.pub = rospy.Publisher(
            name = '/cpz2724_rsw2/di16',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
            )
        pass
    
    def set_error(self):
        self.pub.publish(False)
        return


if __name__=='__main__':
    rospy.init_node(name)
    c = dome_error_sim()
    c.set_error()
    rospy.spin()
