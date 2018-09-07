#! /usr/bin/env python2

name = 'test_dome_door_cmd_sim'

# ----
import time
import random
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeDoorCmdSim(unittest.TestCase):
    def setUp(self):
        self.recv_msg = [None for i in range(4)]
        self.received = [False for i in range(4)]
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        self.travel_time = rospy.get_param('~travel_time')
        
        self.pub = rospy.Publisher(
            name = 'dome_door_cmd2',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.sub_pos_left = rospy.Subscriber(
            name = 'dome_door_leftposition',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = {'index': 0},
            queue_size = 1,
        )
        
        self.sub_act_left = rospy.Subscriber(
            name = 'dome_door_leftaction',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = {'index': 1},
            queue_size = 1,
        )
        
        self.sub_pos_right = rospy.Subscriber(
            name = 'dome_door_rightposition',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = {'index': 2},
            queue_size = 1,
        )
        
        self.sub_act_right = rospy.Subscriber(
            name = 'dome_door_rightaction',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = {'index': 3},
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
    
    def send(self, cmd):
        self.set_received_false()
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
        return
    
    def _test_open(self):
        # 1
        self.send('CLOSE')
        time.sleep(self.travel_time*1.5)
        self.send('OPEN')
        self.assertEqual(self.recv(0).data, 'TRANSIT')
        self.assertEqual(self.recv(1).data, 'MOVE')
        self.assertEqual(self.recv(2).data, 'TRANSIT')
        self.assertEqual(self.recv(3).data, 'MOVE')
        self.set_received_false()        
        self.assertEqual(self.recv(0).data, 'OPEN')
        self.assertEqual(self.recv(1).data, 'STOP')
        self.assertEqual(self.recv(2).data, 'OPEN')
        self.assertEqual(self.recv(3).data, 'STOP')        
        # 2
        self.send('OPEN')
        self.assertRaises(Exception, self.recv, 0)
        self.assertRaises(Exception, self.recv, 1)
        return

    def _test_close(self):
        # 1
        self.send('OPEN')
        time.sleep(self.travel_time*1.5)
        self.send('CLOSE')
        self.assertEqual(self.recv(0).data, 'TRANSIT')
        self.assertEqual(self.recv(1).data, 'MOVE')
        self.assertEqual(self.recv(2).data, 'TRANSIT')
        self.assertEqual(self.recv(3).data, 'MOVE')
        self.set_received_false()
        self.assertEqual(self.recv(0).data, 'CLOSE')
        self.assertEqual(self.recv(1).data, 'STOP')
        self.assertEqual(self.recv(2).data, 'CLOSE')
        self.assertEqual(self.recv(3).data, 'STOP')        
        # 2
        self.send('CLOSE')
        self.assertRaises(Exception, self.recv, 2)
        self.assertRaises(Exception, self.recv, 3)
        return


if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_door_cmd_sim', TestDomeDoorCmdSim)
