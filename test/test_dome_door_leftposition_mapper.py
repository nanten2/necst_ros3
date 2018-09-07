#! /usr/bin/env python2

name = 'test_dome_leftposition_mapper'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeLeftpositionMapper(unittest.TestCase):
    def setUp(self):
        self.recv_msg = None
        self.received = False
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub1 = rospy.Publisher(
            name = 'cpz2724_rsw2_di06',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.pub2 = rospy.Publisher(
            name = 'cpz2724_rsw2_di07',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.sub = rospy.Subscriber(
            name = 'dome_door_leftposition',
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

    def send(self, d1, d2):
        self.received = False
        self.pub1.publish(d1)
        self.pub2.publish(d2)
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
        self._test_open()
        self._test_close()
        self._test_transit()
        return
    
    def _test_open(self):
        self.send(True, True)
        self.assertEqual(self.recv().data, 'OPEN')
        self.send(True, False)
        self.assertEqual(self.recv().data, 'OPEN')
        return

    def _test_close(self):
        self.send(False, True)
        self.assertEqual(self.recv().data, 'CLOSE')
        return

    def _test_transit(self):
        self.send(False, False)
        self.assertEqual(self.recv().data, 'TRANSIT')
        return





if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_leftposition_mapper', TestDomeLeftpositionMapper)
