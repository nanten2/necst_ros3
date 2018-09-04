#! /usr/bin/env python2

name = 'test_dome_emergency'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeEmergency(unittest.TestCase):
    def setUp(self):
        self.recv_msg = None
        self.received = False
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub = rospy.Publisher(
            name = 'dome_emergency_input_sim',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.sub = rospy.Subscriber(
            name = 'dome_emergency',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            queue_size = 1,
        )

        time.sleep(0.5)
        pass

    def callback(self, msg):
        self.recv_msg = msg
        self.received = True
        return

    def send(self, value):
        self.received = False
        self.pub.publish(value)
        return
        
    def recv(self):
        start = time.time()
        while True;:
            if self.received == True:
                break
            if time.time() - start > self.timeout:
                raise Exception('timeout')
            time.sleep(0.001)
            continue
        return self.recv_msg
    
    def test_emergency_on(self):
        self.send(True)
        ret = self.recv()
        self.assertEqual(ret.data, True)
        return

    def test_emergency_off(self):
        self.send(False)
        ret = self.recv()
        self.assertEqual(ret.data, False)
        return



if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_emergency', TestDomeEmergency)
