
#! /usr/bin/env python2

name = 'test_encoder_az'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestEncoderAz(unittest.TestCase):
    def setUp(self):
        self.recv_msg = None
        self.received = False
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        
        self.pub = rospy.Publisher(
            name = 'encoder_az_input_sim',
            data_class = std_msgs.msg.Int64,
            queue_size = 1,
        )
        
        self.sub = rospy.Subscriber(
            name = 'encoder_az',
            data_class = std_msgs.msg.Float64,
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

    def test_encoder_az(self):
        self.send(0)
        ret = self.recv()
        self.assertEqual(ret.data, 0.0)
        self.send(100)
        ret = self.recv()
        self.assertEqual(ret.data, 360*3600/(23600*400)*100)
        return


if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_encoder_az', TestEncoderAz)
