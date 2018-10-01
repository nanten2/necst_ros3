#! /usr/bin/env python3

name = 'drive_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class antenna_drive_sim(object):
    p = {
        'do1': None,
        'do2': None,
        'do3': None,
        'do4': None,
        'do5': None,
        'do6': None,
    }

    cmd = ''
    
    def __init__(self):
        self.do1 = rospy.Subscriber(
            name = '/cpz2724_rsw1/do01',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do1',
            queue_size = 1,
        )
        
        self.do2 = rospy.Subscriber(
            name = '/cpz2724_rsw1/do02',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do2',
            queue_size = 1,
        )
        
        self.do3 = rospy.Subscriber(
            name = '/cpz2724_rsw1/do09',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do3',
            queue_size = 1,
        )
        
        self.do4 = rospy.Subscriber(
            name = '/cpz2724_rsw1/do10',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do4',
            queue_size = 1,
        )
        
        self.do5 = rospy.Subscriber(
            name = '/cpz2724_rsw1/do11',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do5',
            queue_size = 1,
        )
        
        self.do6 = rospy.Subscriber(
            name = '/cpz2724_rsw1/do12',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do6',
            queue_size = 1,
        )
        
        self.pub_din1 = rospy.Publisher(
            name = '/cpz2724_rsw0/di01',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_din2 = rospy.Publisher(
            name = '/cpz2724_rsw0/di02',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_din3 = rospy.Publisher(
            name = '/cpz2724_rsw0/di03',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_din4 = rospy.Publisher(
            name = '/cpz2724_rsw0/di04',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        pass

    def callback(self, msg, argname):
        self.p[argname] = msg.data
        self.callback2()
        return

    def callback2(self):
        if self.p['do1'] == True and self.p['do2'] == True and self.p['do3'] == True and self.p['do4'] == True and self.p['do5'] == True and self.p['do6'] == True:
            self.cmd = 'on'
        elif self.p['do1'] == False and self.p['do2'] == False and self.p['do3'] == False and self.p['do4'] == False and self.p['do5'] == False and self.p['do6'] == False:
            self.cmd = 'off'
        else:
            pass
        return
            
    def publish_status(self):
        cmd_last = ''
        while not rospy.is_shutdown():
            if self.cmd != cmd_last:
                if self.cmd == 'on':
                    self.pub_din1(True)
                    self.pub_din2(True)
                    self.pub_din3(True)
                    self.pub_din4(True)
                elif self.cmd == 'off':
                    self.pub_din1(False)
                    self.pub_din2(False)
                    self.pub_din3(False)
                    self.pub_din4(False)
                cmd_last = self.cmd
                pass
            
            time.sleep(0.1)
            continue

        return


if __name__=='__main__':
    rospy.init_node(name)
    sim = antenna_drive_sim()
    pub_thread = threading.Thread(
        target = sim.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
