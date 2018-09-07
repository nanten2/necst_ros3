#! /usr/bin/env python2

name = 'test_dome_action_mapper'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeActionMapper(unittest.TestCase):
    def setUp(self):
        self.recv_msg = None
        self.received = False
        
        rospy.init_node(name)
        name_topic_to = rospy.get_param('~name_topic_to')
        name_topic_from = rospy.get_param('~name_topic_from')
        
        self.timeout = rospy.get_param('~timeout')
        
        self.pub = rospy.Publisher(
            name = name_topic_from,
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.sub = rospy.Subscriber(
            name = name_topic_to,
            data_class = std_msgs.msg.String,
            callback = self.callback,
            queue_size = 1,
        )

        time.sleep(self.timeout * 0.5)
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
        while True:
            if self.received == True:
                break
            if time.time() - start > self.timeout:
                raise Exception('timeout')
            time.sleep(0.001)
            continue
        return self.recv_msg

    def test_all(self):
        self._test_stop()
        self._test_move()
        return
    
    def _test_stop(self):
        self.send(False)
        ret = self.recv()
        self.assertEqual(ret.data, 'STOP')
        return

    def _test_move(self):
        self.send(True)
        ret = self.recv()
        self.assertEqual(ret.data, 'MOVE')
        return




if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_action_mapper', TestDomeActionMapper)
