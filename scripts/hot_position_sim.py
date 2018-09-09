#!/usr/bin/env python3

name = "hot_position_sim"

import time
import threading
import rospy
import std_msgs.msg


class hot_position_sim(object):

    bit_status = [0,0,0,0,0,0,0,0]

    def __init__(self):

        self.topic_to = []
        for i in range(1,3):
            topic_to_ = rospy.Publisher(
                    name = "hot_cpz2724_rsw0_di%d"%(i),
                    data_class = std_msgs.msg.Byte,
                    queue_size = 1,
                    latch = True
                )
            self.topic_to.append(topic_to_)
            continue


        self.topic_from = []
        for j in range(1,9):
            topic_from_ = rospy.Subscriber(
                    name = 'hot_cpz2724_rsw1_do%d'%(j),
                    data_class = std_msgs.msg.Byte,
                    callback = self.update_bit_status,
                    callback_args = {'index': j-1 },
                    queue_size = 1,
                )
            self.topic_from.append(topic_from_)
            continue

        pass

    def update_bit_status(self, command, args):
        index = args["index"]
        self.bit_status[index] = command.data
        return

    def publish_hot(self):
        byte_last = []
        while not rospy.is_shutdown():
            byte = self.bit_status

            if byte != byte_last:
                if byte == [0,0,0,0,1,0,0,0]:
                    topic_to[0].publish(1)
                    topic_to[1].publish(1)
                    time.sleep(4)
                    topic_to[0].publish(0)
                    topic_to[1].publish(1)
                elif byte == [0,1,0,0,1,1,0,0]:
                    topic_to[0].publish(1)
                    topic_to[1].publish(1)
                    time.sleep(4)
                    topic_to[0].publish(1)
                    topic_to[1].publish(0)
                else:
                    pass

                byte_last = byte
            time.sleep(0.05)
            continue

        return


if __name__ == "__main__":
    rospy.init_node(name)
    hot_sim = hot_position_sim()
    
    pub_thread = threading.Thread(
            target = hot_sim.publish_hot(),
            daemon = True
        )
    pub_thread.start()
    
    rospy.spin()

