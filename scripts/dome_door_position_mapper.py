#! /usr/bin/env python3

name = 'dome_door_position_mapper'

# ----
import time
import threading
import rospy
import std_msgs.msg


class dome_door_position_mapper(object):
    bit_status = [0,0]
    
    def __init__(self):
        name_topic_to = rospy.get_param('~name_topic_to')
        name_topic_from1 = rospy.get_param('~name_topic_from1')
        name_topic_from2 = rospy.get_param('~name_topic_from2')
        
        self.topic_to = rospy.Publisher(
            name = name_topic_to,
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        
        self.topic_from1 = rospy.Subscriber(
            name = name_topic_from1,
            data_class = std_msgs.msg.Bool,
            callback = self.update_bit_status,
            callback_args = {'index': 0},
            queue_size = 1,
        )
        
        self.topic_from1 = rospy.Subscriber(
            name = name_topic_from2,
            data_class = std_msgs.msg.Bool,
            callback = self.update_bit_status,
            callback_args = {'index': 1},
            queue_size = 1,
        )
        pass

    def update_bit_status(self, msg, args):
        self.bit_status[args['index']] = int(msg.data)
        return

    def publish_status(self):
        bit_status_last = self.bit_status.copy()
        
        while not rospy.is_shutdown():
            if self.bit_status != bit_status_last:
                if self.bit_status[0] == 1:
                    status = 'OPEN'
                else:
                    if self.bit_status[1] == 1:
                        status = 'CLOSE'
                    else:
                        status = 'TRANSIT'
                        pass
                    pass
                
                self.topic_to.publish(status)
                bit_status_last = self.bit_status.copy()
                pass

            time.sleep(0.05)
            continue

        return
    


if __name__=='__main__':
    rospy.init_node(name)
    mapper = dome_door_position_mapper()
    pub_thread = threading.Thread(
        target = mapper.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
