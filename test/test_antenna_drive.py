#! /usr/bin/env python2

name = 'test_antenna_drive'

# ----
import time
import unittest
import rospy
import rostest
import std_msgs.msg


class TestAntennaDrive(unittest.TestCase):
    
    received = {
        'drive': False,
        'cmd': False,
        'cmd2': False,
        'lock': False,
    }

    p = {
        'drive': False,
        'cmd': False,
        'cmd2': False,
        'lock': False,
    }

    def setUp(self):
        rospy.init_node(name)
        self.timeout = float(rospy.get_param('~timeout'))
        
        self.pub_emergency = rospy.Publisher(
            name = 'antenna_emergency',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )

        self.pub_control = rospy.Publisher(
            name = 'antenna_control',
            data_class = std_msgs.msg.Bool,
            queue_size = 1,
        )

        self.pub_cmd = rospy.Publisher(
            name = 'drive_cmd',
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )
        
        self.drive = rospy.Subscriber(
            name = 'drive',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = 'drive',
            queue_size = 1,
        )

        self.cmd = rospy.Subscriber(
            name = 'drive_cmd',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = 'cmd',
            queue_size = 1,
        )

        self.cmd2 = rospy.Subscriber(
            name = 'drive_cmd2',
            data_class = std_msgs.msg.String,
            callback = self.callback,
            callback_args = 'cmd2',
            queue_size = 1,
        )

        self.lock = rospy.Subscriber(
            name = 'drive_lock',
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
        self._test_off()
        self._test_on()
        self._test_emergency()
        self._test_control()
        return
    
    def _test_off(self):
        # initialize
        self.send_emergency(False)
        self.send_control('REMOTE')
        self.send('on')
        time.sleep(self.timeout * 1.5)

        self.send('off')
        time.sleep(self.timeout * 0.2)
        self.assertEqual(self.recv('cmd2'), 'off')
        self.assertEqual(self.recv('drive'), 'off')
        return
    
    def _test_on(self):
        # initialize
        self.send_emergency(False)
        self.send_control('REMOTE')
        self.send('off')
        time.sleep(self.timeout * 1.5)

        self.send('on')
        time.sleep(self.timeout * 0.2)
        self.assertEqual(self.recv('cmd2'), 'on')
        self.assertEqual(self.recv('door'), 'on')
        return
    
    def _test_emergency(self):
        # initialize
        self.send_emergency(False)
        self.send_control('REMOTE')
        self.send('on')
        time.sleep(self.timeout * 1.5)

        # switch on emergency
        self.send_emergency(True)
        time.sleep(self.timeout * 0.5)
        self.assertEqual(self.recv('cmd'), 'on')
        self.assertEqual(self.recv('cmd2'), 'on')
        self.assertEqual(self.recv('lock'), True)
        
        self.send('off')
        self.assertRaises(Exception, self.recv, 'cmd2')
        self.assertEqual(self.recv('cmd'), 'off')
        
        # switch off emergency
        self.send_emergency(False)
        self.assertEqual(self.recv('lock'), False)
        self.assertEqual(self.recv('cmd'), 'off')
        time.sleep(self.timeout * 1.1)
        self.assertEqual(self.recv('drive'), 'off')
        return

    def _test_control(self):
        # initialize
        self.send_emergency(False)
        self.send_control('REMOTE')
        self.send('on')
        time.sleep(self.timeout * 1.5)

        self.send_control('LOCAL')
        time.sleep(self.timeout * 0.5)
        self.assertEqual(self.recv('cmd'), 'on')
        self.assertEqual(self.recv('cmd2'), 'on')
        self.assertEqual(self.recv('lock'), True)
        
        self.send('off')
        self.assertRaises(Exception, self.recv, 'cmd2')
        self.assertEqual(self.recv('cmd'), 'off')
        
        self.send_control('REMOTE')
        self.assertEqual(self.recv('lock'), False)
        self.assertEqual(self.recv('cmd'), 'off')
        time.sleep(self.timeout * 1.1)
        self.assertEqual(self.recv('drive'), 'off')
        return


if __name__=='__main__':
    rostest.rosrun('necst_ros3', 'test_antenna_drive', TestAntennaDrive)
