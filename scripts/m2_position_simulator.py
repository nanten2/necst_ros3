#! /usr/bin/env python3

name = 'position_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class m2_position_sim(object):
    cmd = 0
    
    def __init__(self):
        self.do = rospy.Subscriber(
            name = '/cpz2724_rsw1/pulse',
            data_class = std_msgs.msg.Float64,
            callback = self.callback,
            queue_size = 1,
        )
        
        self.pub = rospy.Publisher(
            name = '/cpz2724_rsw1/m_pos',
            data_class = std_msgs.msg.Float64,
            latch = True,
            queue_size = 1,
        )
        
        self.cmd = 0.0
        pass

    def callback(self, req):
        self.cmd = req.data /80
        return
            
    def publish_status(self):
        cmd_last = None
        while not rospy.is_shutdown():
            if self.cmd != cmd_last:
                hensa = cmd_last - self.cmd
                for i in range(5):
                    cmd_last += hensa/5
                    self.pub.publish(cmd_last)
                    time.sleep(1)
                cmd_last = self.cmd
                pass
            
            time.sleep(0.1)
            continue

        return


if __name__=='__main__':
    rospy.init_node(name)
    sim = m2_position_sim()
    pub_thread = threading.Thread(
        target = sim.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
