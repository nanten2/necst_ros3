#! /usr/bin/env python2

name = 'test_dome_door_wathcer_finalize_move'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeDoorWatcherFinalizeMove(unittest.TestCase):
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
            name = 'dome_door_cmd2',
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

    def send(self, left, right):
        self.received = False
        self.pub1.publish(left)
        self.pub2.publish(right)
        time.sleep(self.timeout * 0.2)
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
        return
    
    def _test_open(self):
        self.send('TRANSIT', 'TRANSIT')
        self.send('OPEN', 'TRANSIT')
        self.assertRaises(Exception, self.recv)
        self.send('OPEN', 'OPEN')
        self.assertEqual(self.recv().data, 'STOP')
        return

    def _test_close(self):
        self.send('TRANSIT', 'TRANSIT')
        self.send('TRANSIT', 'CLOSE')
        self.assertRaises(Exception, self.recv)
        self.send('CLOSE', 'CLOSE')
        self.assertEqual(self.recv().data, 'STOP')
        return



if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_door_watcher_finalize_moce', TestDomeDoorWatcherFinalizeMove)
