#! /usr/bin/env python3

name = 'dome_door_lock'

# ----
import time
import threading
import rospy
import std_msgs.msg


class dome_door_locker(object):
    status = {
        'emergency': None,
        'control': None,
    }

    output = {
        'lock': False,
        'cmd': None,
        'cmd2': None,
    }
    publish_output = False
    
    def __init__(self):
        self.pub_lock = rospy.Publisher(
            name = 'dome_door_lock',
            data_class = std_msgs.msg.Bool,
            latch = True,
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

        self.topic_from = []
        
        topic_from1 = rospy.Subscriber(
            name = 'dome_emergency',
            data_class = std_msgs.msg.Bool,
            callback = self.update_emergency,
            queue_size = 1,
        )
        self.topic_from.append(topic_from1)
        
        topic_from2 = rospy.Subscriber(
            name = 'dome_control',
            data_class = std_msgs.msg.String,
            callback = self.update_control,
            queue_size = 1,
        )
        self.topic_from.append(topic_from2)
        pass
    
    def update_emergency(self, msg):
        self.status['emergency'] = msg.data
        self.judge()
        return

    def update_control(self, msg):
        self.status['control'] = msg.data
        self.judge()
        return
    
    def judge(self):
        is_normal_state = True
        
        if self.status['emergency'] == True:
            is_normal_state = False
            self.output['cmd'] = 'STOP'
            self.output['cmd2'] = 'STOP'
            self.output['lock'] = True
            self.publish_output = True
            pass
        
        if self.status['control'] == 'LOCAL':
            is_normal_state = False
            self.output['cmd'] = 'STOP'
            self.output['cmd2'] = 'STOP'
            self.output['lock'] = True
            self.publish_output = True
            pass
        
        if is_normal_state == True:
            self.output['cmd'] = 'STOP'
            self.output['cmd2'] = 'STOP'
            self.output['lock'] = False
            self.publish_output = True
            pass
            
        pass
    
    def publish_status(self):
        while not rospy.is_shutdown():

            if self.publish_output == True:
                self.pub_cmd.publish(self.output['cmd'])
                self.pub_cmd2.publish(self.output['cmd2'])
                self.pub_lock.publish(self.output['lock'])
                self.publish_output = False
                pass
            
            time.sleep(0.05)
            continue

        return
    


if __name__=='__main__':
    rospy.init_node(name)
    locker = dome_door_locker()
    pub_thread = threading.Thread(
        target = locker.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
