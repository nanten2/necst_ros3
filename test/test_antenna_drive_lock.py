#! /usr/bin/env python2

name = 'test_antenna_drive_lock'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestAntennaDrive(unittest.TestCase):
    def setUp(self):
        self.recv_msg = [None for i in range(2)]
        self.received = [False for i in range(2)]

        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub1 = rospy.Publisher(
            name = 'antenna_emergency',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.pub2 = rospy.Publisher(
            name = 'antenna_control',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.pub_cmd = rospy.Publisher(
            name = 'antenna_drive_cmd',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.sub_lock = rospy.Subscriber(
            name = 'antenna_drive_lock',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = {'index': 0},
            queue_size = 1,
        )

        self.sub_cmd = rospy.Subscriber(
            name = 'antenna_drive_cmd',
            data_class = std_msgs.msg.String,
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

    def test_all(self):
        self._test_emergency()
        self._test_control()
        return
    
    def _test_emergency(self):
        self.send(False, 'REMOTE')
        time.sleep(self.timeout * 0.3)
        self.send(True, 'REMOTE')
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, 'off')
        time.sleep(self.timeout * 0.3)
        self.send(True, 'LOCAL')
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, 'off')
        time.sleep(self.timeout * 0.3)
        time.sleep(self.timeout * 0.3)
        self.send(False, 'REMOTE')
        self.pub_cmd.publish('on')
        self.assertEqual(self.recv(0).data, False)
        self.assertEqual(self.recv(1).data, 'on')
        return

    def _test_control(self):
        self.send(False, 'REMOTE')
        time.sleep(self.timeout * 0.3)
        self.send(False, 'LOCAL')
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, 'off')
        time.sleep(self.timeout * 0.3)
        self.send(True, 'LOCAL')
        self.assertEqual(self.recv(0).data, True)
        self.assertEqual(self.recv(1).data, 'off')
        time.sleep(self.timeout * 0.3)
        time.sleep(self.timeout * 0.3)
        self.send(False, 'REMOTE')
        self.pub_cmd.publish('on')
        self.assertEqual(self.recv(0).data, False)
        self.assertEqual(self.recv(1).data, 'on')
        return


if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_antenna_drive', TestAntennaDrive)
