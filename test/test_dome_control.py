#! /usr/bin/env python2

name = 'test_dome_control'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeControl(unittest.TestCase):
    def setUp(self):
        self.recv_msg = None
        self.received = False
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub = rospy.Publisher(
            name = 'dome_control_input_sim',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.sub = rospy.Subscriber(
            name = 'dome_control',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            queue_size = 1,
        )
        pass

    def callback(self, msg):
        self.recv_msg = msg
        self.received = True
        return

    def recv(self):
        start = time.time()
        while (self.received == False and
               time.time() - start < self.timeout):
            time.sleep(0.001)
            continue
        return self.recv_msg
    
    def test_control_local(self):
        self.pub.publish(True)
        ret = self.recv()
        self.assertEqual(ret.data, 'LOCAL')
        return

    def test_control_remote(self):
        self.pub.publish(False)
        ret = self.recv()
        self.assertEqual(ret.data, 'REMOTE')
        return



if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_control', TestDomeControl)
