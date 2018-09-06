#! /usr/bin/env python2

name = 'test_dome_door'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeDoor(unittest.TestCase):
    def setUp(self):
        self.recv_msg = None
        self.received = False
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub1 = rospy.Publisher(
            name = 'dome_door_leftposition',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.pub2 = rospy.Publisher(
            name = 'dome_door_rightposition',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.sub = rospy.Subscriber(
            name = 'dome_door',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            queue_size = 1,
        )

        time.sleep(0.5)
        pass

    def callback(self, msg):
        self.recv_msg = msg
        self.received = True
        return

    def send(self, value1, value2):
        self.received = False
        self.pub1.publish(value1)
        self.pub2.publish(value2)
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
    
    def test_open(self):
        self.send('OPEN', 'OPEN')
        ret = self.recv()
        self.assertEqual(ret.data, 'OPEN')
        return

    def test_close(self):
        self.send('CLOSE', 'CLOSE')
        ret = self.recv()
        self.assertEqual(ret.data, 'CLOSE')
        return

    def test_transit(self):
        # Combination 1
        self.send('TRANSIT', 'OPEN')
        ret = self.recv()
        self.assertEqual(ret.data, 'TRANSIT')
        # Combination 2
        self.send('TRANSIT', 'CLOSE')
        ret = self.recv()
        self.assertEqual(ret.data, 'TRANSIT')
        # Combination 3
        self.send('OPEN', 'TRANSIT')
        ret = self.recv()
        self.assertEqual(ret.data, 'TRANSIT')
        # Combination 4
        self.send('CLOSE', 'TRANSIT')
        ret = self.recv()
        self.assertEqual(ret.data, 'TRANSIT')
        return

    def test_error(self):
        # Combination 1
        self.send('CLOSE', 'OPEN')
        ret = self.recv()
        self.assertEqual(ret.data, 'ERROR')
        # Combination 2
        self.send('OPEN', 'CLOSE')
        ret = self.recv()
        self.assertEqual(ret.data, 'ERROR')
        return


if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_door', TestDomeDoor)
