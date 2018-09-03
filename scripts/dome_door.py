#! /usr/bin/env python3

name = 'dome_door'

# ----
import time
import threading
import rospy
import std_msgs.msg


class dome_door_mapper(object):
    position_left = ''
    position_right = ''
    
    def __init__(self):
        self.topic_to = rospy.Publisher(
            name = name,
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        
        self.topic_from = []
        
        topic_from1 = rospy.Subscriber(
            name = 'dome_door_left_position',
            data_class = std_msgs.msg.String,
            callback = self.update_position_left,
            queue_size = 1,
        )
        self.topic_from.append(topic_from1)
        
        topic_from2 = rospy.Subscriber(
            name = 'dome_door_right_position',
            data_class = std_msgs.msg.String,
            callback = self.update_position_right,
            queue_size = 1,
        )
        self.topic_from.append(topic_from2)

        pass
    
    def update_position_left(self, msg):
        self.position_left = msg.data
        return

    def update_position_right(self, msg):
        self.position_right = msg.data
        return

    def publish_status(self):
        position_left_last = self.position_left
        position_right_last = self.position_right
        
        while not rospy.is_shutdown():
            
            if (self.position_left != position_left_last) \
               or (self.position_right != position_right_last):
                
                if self.position_left == self.position_right == 'OPEN':
                    status = 'OPEN'
                    
                elif self.position_left == self.position_right == 'CLOSE':
                    status = 'CLOSE'

                elif 'MOVING' in [self.position_left, self.position_right]:
                    status = 'MOVING'

                else:
                    status = 'ERROR'
                    pass

                self.topic_to.publish(status)
                position_left_last = self.position_left
                position_right_last = self.position_right
                pass
            
            time.sleep(0.05)
            continue

        return
    


if __name__=='__main__':
    rospy.init_node(name)
    mapper = dome_door_mapper()
    pub_thread = threading.Thread(
        target = mapper.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
