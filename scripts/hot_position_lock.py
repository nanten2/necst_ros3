#! /usr/bin/env python3

name = "hot_position_lock"
target_name = "hot_position"

# ----
import time
import threading
import rospy
import std_msgs.msg


class hot_position_locker(object):
    status = {
    }

    output = {
        'lock': False,
        'cmd': None,
    }
    publish_output = False
    
    def __init__(self):
        self.pub_lock = rospy.Publisher(
            name = target_name +"_lock",
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )

        self.pub_cmd = rospy.Publisher(
            name = target_name +"_cmd",
            data_class = std_msgs.msg.String,
            queue_size = 1,
        )

        topic_from = rospy.Subscriber(
            name = ,
            data_class = std_msgs.msg.Bool,
            callback = self.update_
            queue_size = 1,
        )
        
        pass
    
    def update_(self, msg):
        self.status[''] = msg.data
        self.judge()
        return

    def judge(self):
        is_normal_state = True
        
        if self.status[''] == True:
            is_normal_state = False
            self.output['cmd'] = ''
            self.output['lock'] = True
            self.publish_output = True
            pass
        
        if is_normal_state == True:
            self.output['lock'] = False
            self.publish_output = True
            pass
            
        pass
    
    def publish_status(self):
        self.pub_lock.publish(self.output['lock'])
        
        while not rospy.is_shutdown():

            if self.publish_output == True:
                self.pub_cmd.publish(self.output['cmd'])
                self.pub_lock.publish(self.output['lock'])
                self.publish_output = False
                pass
            
            time.sleep(0.05)
            continue

        return
    


if __name__=='__main__':
    rospy.init_node(name)
    locker = hot_position_locker()
    pub_thread = threading.Thread(
        target = locker.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()

