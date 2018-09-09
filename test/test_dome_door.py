#! /usr/bin/env python2

name = 'test_dome_door'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestDomeDoor(unittest.TestCase):
    received = {
        'door': False,
        'cmd2': False,
    }
    
    p = {
        'door': None,
        'cmd2': None,
    }
    
    def setUp(self):
        rospy.init_node(name)
        self.timeout = rospy.get_param('~timeout')
        self.travel_time = rospy.get_param('~travel_time')
        
        self.pub_emergency = rospy.Publisher(
            name = '/cpz2724_rsw2/di01',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_control = rospy.Publisher(
            name = '/cpz2724_rsw2/di11',
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_cmd = rospy.Publisher(
            name = 'door_cmd',
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        
        self.door = rospy.Subscriber(
            name = 'door',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = 'door',
            queue_size = 1,
        )
        
        self.cmd = rospy.Subscriber(
            name = 'door_cmd',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = 'cmd',
            queue_size = 1,
        )

        self.cmd2 = rospy.Subscriber(
            name = 'door_cmd2',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = 'cmd2',
            queue_size = 1,
        )

        self.lock = rospy.Subscriber(
            name = 'door_lock',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'lock',
            queue_size = 1,
        )

        time.sleep(self.timeout * 0.5)
        pass

    def callback(self, msg, argname):
        self.p[argname] = msg.data
        self.received[argname] = True
        return

    def set_received_false(self):
        for key in self.received.keys():
            self.received[key] = False
            continue
        return
    
    def recv(self, key):
        start = time.time()
        while True:
            if self.received[key] == True:
                break
            if time.time() - start > self.timeout:
                raise Exception('timeout')
            time.sleep(0.001)
            continue
        return self.p[key]

    def send(self, cmd):
        self.set_received_false()
        self.pub_cmd.publish(cmd)
        return

    def send_emergency(self, emergency):
        self.set_received_false()
        self.pub_emergency.publish(emergency)
        return

    def send_control(self, ctrl):
        self.set_received_false()
        if ctrl == 'LOCAL':
            self.pub_control.publish(True)
        else:
            self.pub_control.publish(False)
            pass
        return

    def test_all(self):
        self._test_close()
        self._test_open()
        self._test_emergency()
        self._test_control()
        return
    
    def _test_close(self):
        # initialize
        self.send_emergency(False)
        self.send_control('REMOTE')
        self.send('OPEN')
        time.sleep(self.travel_time * 1.5)

        # start closing
        self.send('CLOSE')
        # moving
        self.assertEqual(self.recv('cmd2'), 'CLOSE')
        self.assertEqual(self.recv('door'), 'TRANSIT')
        # close / stop
        self.received = False
        self.assertEqual(self.recv('cmd2'), 'STOP')
        self.assertEqual(self.recv('door'), 'CLOSE')
        return
    
    def _test_open(self):
        # initialize
        self.send_emergency(False)
        self.send_control('REMOTE')
        self.send('CLOSE')
        time.sleep(self.travel_time * 1.5)

        # start opening
        self.send('OPEN')
        # moving
        self.assertEqual(self.recv('cmd2'), 'OPEN')
        self.assertEqual(self.recv('door'), 'TRANSIT')
        # open / stop
        self.received = False
        self.assertEqual(self.recv('cmd2'), 'STOP')
        self.assertEqual(self.recv('door'), 'OPEN')
        return
    
    def _test_emergency(self):
        # initialize
        self.send_emergency(False)
        self.send_control('REMOTE')
        self.send('OPEN')
        time.sleep(self.travel_time * 1.5)

        # switch on emergency when closing
        self.send('CLOSE')
        self.assertEqual(self.recv('door'), 'TRANSIT')
        
        self.send_emergency(True)
        self.assertEqual(self.recv('cmd'), 'STOP')
        self.assertEqual(self.recv('cmd2'), 'STOP')
        self.assertEqual(self.recv('lock'), True)
        
        self.send('CLOSE')
        self.assertRaises(Exception, self.recv, 'cmd2')
        self.assertEqual(self.recv('cmd'), 'CLOSE')
        
        # switch off emergency
        self.send_emergency(False)
        self.assertEqual(self.recv('cmd'), 'STOP')
        
        self.send('CLOSE')
        self.assertEqual(self.recv('door'), 'CLOSE')
        return

    def _test_control(self):
        # initialize
        self.send_emergency(False)
        self.send_control('REMOTE')
        self.send('OPEN')
        time.sleep(self.travel_time * 1.5)

        # change to LOCAL mode when closing
        self.send('CLOSE')
        self.assertEqual(self.recv('door'), 'TRANSIT')
        
        self.send_control('LOCAL')
        self.assertEqual(self.recv('cmd'), 'STOP')
        self.assertEqual(self.recv('cmd2'), 'STOP')
        self.assertEqual(self.recv('lock'), True)
        
        self.send('CLOSE')
        self.assertRaises(Exception, self.recv, 'cmd2')
        self.assertEqual(self.recv('cmd'), 'CLOSE')
        
        # change to REMOTE mode
        self.send_control('LOCAL')
        self.assertEqual(self.recv('cmd'), 'STOP')
        
        self.send('CLOSE')
        self.assertEqual(self.recv('door'), 'CLOSE')
        return



if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_dome_door', TestDomeDoor)
