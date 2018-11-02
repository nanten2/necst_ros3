#! /usr/bin/env python3

name = 'position_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class m4_position_sim(object):
    cmd = ''
    
    def __init__(self):
        self.do = rospy.Subscriber(
            name = '/cpz7204_rsw0/step',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            queue_size = 1,
        )
        
        self.pub_busy = rospy.Publisher(
            name = '/cpz7204_rsw0/busy',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_p = rospy.Publisher(
            name = '/cpz7204_rsw0/p_EL',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_m = rospy.Publisher(
            name = '/cpz7204_rsw0/m_EL',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.cmd = "NASCO"
        pass

    def callback(self, req):
        if req.data == True:
            self.cmd = "NASCO"
        else:
            self.cmd = "SMART"
        return
            
    def publish_status(self):
        cmd_last = ''
        while not rospy.is_shutdown():
            if self.cmd != cmd_last:
                if self.cmd == 'NASCO':
                    self.pub_busy.publish(True)
                    time.sleep(4)
                    self.pub_p.publise(True)
                    self.pub_m.publish(False)
                elif self.cmd == 'SMART':
                    self.pub_busy.publish(True)
                    time.sleep(4)
                    self.pub_p.publise(False)
                    self.pub_m.publish(True)
                cmd_last = self.cmd
                pass
            
            time.sleep(0.1)
            continue

        return


if __name__=='__main__':
    rospy.init_node(name)
    sim = m4_position_sim()
    pub_thread = threading.Thread(
        target = sim.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
