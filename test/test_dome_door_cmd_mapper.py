#! /usr/bin/env python2

name = 'test_dome_door_cmd_mapper'

# ----
import time
import random
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeDoorCmdMapper(unittest.TestCase):
    def setUp(self):
        self.recv_msg = [None for i in range(2)]
        self.received = [False for i in range(2)]
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub = rospy.Publisher(
            name = 'dome_door_cmd2',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.pub_lock = rospy.Publisher(
            name = 'dome_door_lock',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
            latch = True,
        )
        
        self.sub1 = rospy.Subscriber(
            name = 'cpz2724_rsw2_do05',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = {'index': 0},
            queue_size = 1,
        )
        
        self.sub2 = rospy.Subscriber(
            name = 'cpz2724_rsw2_do06',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = {'index': 1},
            queue_size = 1,
        )
        
        time.sleep(self.timeout * 0.5)
        pass

    def callback(self, msg, args):
        self.recv_msg[args['index']] = msg
        self.received[args['index']] = True
        return

    def set_received_false(self):
        for i in range(len(self.received)):
            self.received[i] = False
            continue
        return
    
    def send(self, cmd, lock):
        self.set_received_false()
        self.pub_lock.publish(lock)
        time.sleep(self.timeout * 0.2)
        self.pub.publish(cmd)
        return
        
    def recv(self, index):
        start = time.time()
        while True:
            if self.received[index] == True:
                break
            if time.time() - start > self.timeout:
                raise Exception('timeout')
            time.sleep(0.001)
            continue
        return self.recv_msg[index]

    def test_all(self):
        self._test_open()
        self._test_close()
        self._test_stop()
        return
    
    def _test_open(self):
        # 1
        self.send('OPEN', False)
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, True)
        # 2
        self.send('OPEN', True)
        self.assertRaises(Exception, self.recv, 0)
        return

    def _test_close(self):
        # 1
        self.send('CLOSE', False)
        self.assertEqual(self.recv(0).data, False)
        self.assertEqual(self.recv(1).data, True)
        # 2
        self.send('CLOSE', True)
        self.assertRaises(Exception, self.recv, 1)
        return

    def _test_stop(self):
        # 1
        self.send('STOP', False)
        self.assertEqual(self.recv(0).data, False)
        self.assertEqual(self.recv(1).data, False)
        # 2
        self.send('STOP', True)
        self.assertRaises(Exception, self.recv, 0)
        return


if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_door_cmd_mapper', TestDomeDoorCmdMapper)
