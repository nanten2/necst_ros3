#! /usr/bin/env python2

name = 'test_antenna_drive'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestAntennaDrive(unittest.TestCase):
    def setUp(self):
        self.recv_msg = None
        self.received = False
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub = rospy.Publisher(
            name = 'antenna_drive_cmd',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.sub = rospy.Subscriber(
            name = 'antenna_drive',
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
        self._test_drive_on()
        self._test_drive_off()
        return
    
    def _test_drive_on(self):
        self.send("on")
        ret = self.recv()
        self.assertEqual(ret.data, 'on')
        return

    def _test_drive_off(self):
        self.send("off")
        ret = self.recv()
        self.assertEqual(ret.data, 'off')
        return



if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_antenna_drive', TestAntennaDrive)
