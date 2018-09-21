#! /usr/bin/env python2

name = 'test_dome_door_integ_sim'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeDoorIntegSim(unittest.TestCase):
    def setUp(self):
        self.recv_msg = None
        self.received = False
        
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        self.travel_time = rospy.get_param('~travel_time')
        
        self.pub_emergency = rospy.Publisher(
            name = 'dome_emergency_input_sim',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )
        
        self.pub_control = rospy.Publisher(
            name = 'dome_control_input_sim',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.pub_cmd = rospy.Publisher(
            name = 'dome_door_cmd',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.sub = rospy.Subscriber(
            name = 'dome_door',
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

    def send(self, cmd):
        self.received = False
        self.pub_cmd.publish(cmd)
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
        self._test_close()
        self._test_open()
        self._test_emergency()
        self._test_control()
        return
    
    def _test_close(self):
        self.send('OPEN')
        time.sleep(self.travel_time * 1.5)
        self.send('CLOSE')
        self.assertEqual(self.recv().data, 'TRANSIT')
        self.received = False
        self.assertEqual(self.recv().data, 'CLOSE')
        return
    
    def _test_open(self):
        self.send('CLOSE')
        time.sleep(self.travel_time * 1.5)
        self.send('OPEN')
        self.assertEqual(self.recv().data, 'TRANSIT')
        self.received = False
        self.assertEqual(self.recv().data, 'OPEN')
        return
    
    def _test_emergency(self):
        self.send('OPEN')
        time.sleep(self.travel_time * 1.5)
        
        self.send('CLOSE')
        self.assertEqual(self.recv().data, 'TRANSIT')
        
        self.pub_emergency.publish(True)
        self.send('CLOSE')
        self.assertRaises(Exception, self.recv)
        
        self.received = False
        self.pub_emergency.publish(False)
        self.assertRaises(Exception, self.recv)
        
        self.send('CLOSE')
        self.assertEqual(self.recv().data, 'CLOSE')
        return

    def _test_control(self):
        self.send('CLOSE')
        time.sleep(self.travel_time * 1.5)
        
        self.send('OPEN')
        self.assertEqual(self.recv().data, 'TRANSIT')
        
        self.pub_emergency.publish(True)
        self.send('OPEN')
        self.assertRaises(Exception, self.recv)
        
        self.received = False
        self.pub_emergency.publish(False)
        self.assertRaises(Exception, self.recv)
        
        self.send('OPEN')
        self.assertEqual(self.recv().data, 'OPEN')
        return



if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_door_integ_sim', TestDomeDoorIntegSim)
