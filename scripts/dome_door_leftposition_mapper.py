#! /usr/bin/env python3

name = 'dome_door_leftposition'

# ----
import time
import threading
import rospy
import std_msgs.msg


class dome_door_left_position_mapper(object):
    bit_status = [0,0]
    
    def __init__(self):
        self.topic_to = rospy.Publisher(
            name = name,
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        
        self.topic_from = []
        for i, dioch in enumerate([6,7]):
            topic_from_ = rospy.Subscriber(
                name = 'cpz2724_rsw2_di%d'%(dioch),
                data_class = std_msgs.msg.Bool,
                callback = self.update_bit_status,
                callback_args = {'bit': i},
                queue_size = 1,
            )
            self.topic_from.append(topic_from_)
            continue
        pass

    def update_bit_status(self, msg, args):
        bitindex = args['bit']
        self.bit_status[bitindex] = int(msg.data)
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
                        status = 'MOVING'
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
    mapper = dome_door_left_position_mapper()
    pub_thread = threading.Thread(
        target = mapper.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
