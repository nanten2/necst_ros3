#! /usr/bin/env python2

name = 'test_dome_door_lock'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeDoorLock(unittest.TestCase):
    def setUp(self):
        self.recv_msg = [None for i in range(3)]
        self.received = [False for i in range(3)]
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub1 = rospy.Publisher(
            name = 'dome_emergency',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.pub2 = rospy.Publisher(
            name = 'dome_control',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.pub_cmd = rospy.Publisher(
            name = 'dome_door_cmd',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.pub_cmd2 = rospy.Publisher(
            name = 'dome_door_cmd2',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.sub_lock = rospy.Subscriber(
            name = 'dome_door_lock',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = {'index': 0},
            queue_size = 1,
        )

        self.sub_cmd = rospy.Subscriber(
            name = 'dome_door_cmd',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = {'index': 1},
            queue_size = 1,
        )

        self.sub_cmd2 = rospy.Subscriber(
            name = 'dome_door_cmd2',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = {'index': 2},
            queue_size = 1,
        )

        time.sleep(0.5)
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
    
    def send(self, emergency, control):
        self.set_received_false()
        self.pub1.publish(emergency)
        self.pub2.publish(control)
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
    
    def test_emergency(self):
        self.send(False, 'REMOTE')
        time.sleep(0.1)
        self.send(True, 'REMOTE')
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, 'STOP')
        self.assertEqual(self.recv(2).data, 'STOP')
        time.sleep(0.1)
        self.send(True, 'LOCAL')
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, 'STOP')
        self.assertEqual(self.recv(2).data, 'STOP')
        time.sleep(0.1)
        self.pub_cmd.publish('OPEN')
        self.pub_cmd2.publish('OPEN')
        time.sleep(0.1)
        self.send(False, 'REMOTE')
        self.assertEqual(self.recv(0).data, False)
        self.assertEqual(self.recv(1).data, 'STOP')
        self.assertEqual(self.recv(2).data, 'STOP')
        return

    def test_control(self):
        self.send(False, 'REMOTE')
        time.sleep(0.1)
        self.send(False, 'LOCAL')
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, 'STOP')
        self.assertEqual(self.recv(2).data, 'STOP')        
        time.sleep(0.1)
        self.send(True, 'LOCAL')
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, 'STOP')
        self.assertEqual(self.recv(2).data, 'STOP')        
        time.sleep(0.1)
        self.pub_cmd.publish('OPEN')
        self.pub_cmd2.publish('OPEN')
        time.sleep(0.1)
        self.send(False, 'REMOTE')
        self.assertEqual(self.recv(0).data, False)
        self.assertEqual(self.recv(1).data, 'STOP')
        self.assertEqual(self.recv(2).data, 'STOP')        
        return



if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_door_lock', TestDomeDoorLock)
