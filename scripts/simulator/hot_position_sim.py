#!/usr/bin/env python3

name = "hot_position_sim"

import time
import threading
import rospy
import std_msgs.msg


class hot_position_sim(object):

    bit_status = [1,1,1,1]

    def __init__(self):

        self.topic_to = [rospy.Publisher(
                    name = "/necopt/cpz2724_rsw0/di0%d"%(i),
                    data_class = std_msgs.msg.Bool,
                    queue_size = 1,
                    latch = True
                ) for i in range(1,3)]

        self.topic_from = [rospy.Subscriber(
                    name = '/necopt/cpz2724_rsw0/do0%d'%(j),
                    data_class = std_msgs.msg.Bool,
                    callback = self.update_bit_status,
                    callback_args = {'index': j-1 },
                    queue_size = 1,
                ) for j in range(1,5)]

        pass

    def update_bit_status(self, command, args):
        index = args["index"]
        self.bit_status[index] = command.data
        return

    def publish_hot(self):
        self.topic_to[0].publish(0)
        self.topic_to[1].publish(1)
        while not rospy.is_shutdown():
            byte = self.bit_status

            if byte == [0,0,0,0] or byte == [1,0,0,0]:
                self.topic_to[0].publish(1)
                self.topic_to[1].publish(1)
                time.sleep(2)
                self.topic_to[0].publish(0)
                self.topic_to[1].publish(1)
            elif byte == [0,1,0,0] or byte == [1,1,0,0]:
                self.topic_to[0].publish(1)
                self.topic_to[1].publish(1)
                time.sleep(2)
                self.topic_to[0].publish(1)
                self.topic_to[1].publish(0)
            else:
                pass

            self.bit_status = [1,1,1,1]
            time.sleep(0.1)
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

