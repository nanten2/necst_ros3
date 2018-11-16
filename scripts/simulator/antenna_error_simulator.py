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
        self.pub1 = [rospy.Publisher(
            name = '/necctrl/cpz2724_rsw0/di0{}'.format(i),
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
            ) for i in range(5,10)]

        self.pub2 = [rospy.Publisher(
            name = '/necctrl/cpz2724_rsw0/di{}'.format(i),
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
            ) for i in range(10,25)]
        pass
    
    def set_error(self):
        [self.pub1[i].publish(False) for i in range(5)]
        [self.pub2[i].publish(False) for i in range(15)]
        return


if __name__=='__main__':
    rospy.init_node(name)
    c = antenna_error_sim()
    c.set_error()
    rospy.spin()
