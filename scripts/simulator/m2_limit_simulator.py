#! /usr/bin/env python3

name = 'm2_limit_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class m2_limit_sim(object):
    
    def __init__(self):
        self.pub15 = rospy.Publisher(
            name = '/cpz2724_rsw1/di15',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub16 = rospy.Publisher(
            name = '/cpz2724_rsw1/di16',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )

        pass
    
    def no_limit(self):
        self.pub15.publish(False)
        self.pub16.publish(False)
        return

    def up_limit(self):
        self.pub15.publish(True)
        self.pub16.publish(False)
        return

    def down_limit(self):
        self.pub15.publish(False)
        self.pub16.publish(True)
        return


if __name__=='__main__':	
    rospy.init_node(name)
    c = m2_limit_sim()
    c.no_limit()
    rospy.spin()


