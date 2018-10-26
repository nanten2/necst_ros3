#! /usr/bin/env python3

name = 'm2_emergency_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class m2_emergency_sim(object):
    
    def __init__(self):
        self.pub = rospy.Publisher(
            name = '/cpz2724_rsw1/di15',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        pass
    
    def on_emergency(self):
        self.pub.publish(1)
        return

    def off_emergency(self):
        self.pub.publish(0)
        return


if __name__=='__main__':	
    rospy.init_node(name)
    c = m2_emergency_sim()
    c.on_emergency()
    rospy.spin()


