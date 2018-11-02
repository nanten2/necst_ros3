#! /usr/bin/env python3

name = 'antenna_error_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg



class antenna_error_sim(object):
    p = {}
    
    def __init__(self):
        self.pub = rospy.Publisher(
            name = '/necctrl/cpz2724_rsw0/di05',
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
    c = antenna_error_sim()
    c.set_error()
    rospy.spin()
