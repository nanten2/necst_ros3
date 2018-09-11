#!/usr/bin/env python3

name = "hot_position"

import time
import threading
import rospy
import std_msgs.msg
import topic_utils


class hot_position(object):
    bit_status = [0,0]

    def __init__(self):
        self.topic_to = rospy.Publisher(
                name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
                latch = True
            )

        self.topic_to1 = []
        for i in range(1,9):
            topic_to1_ = rospy.Publisher(
                name = "hot_cpz2724_rsw1_do%d"%(i),
                data_class = std_msgs.msg.Byte,
                queue_size = 1,
                latch = True
            )
            self.topic_to1.append(topic_to1_)
            continue

        self.topic_from = rospy.Subscriber(
                name = name + "_cmd",
                data_class = std_msgs.msg.String,
                callback = self.hot_position_cmd,
                queue_size = 1,
            )

        self.topic_from1 = []
        for j in range(1,3):
            topic_from1_ = rospy.Subscriber(
                name = 'hot_cpz2724_rsw0_di%d'%(j),
                data_class = std_msgs.msg.Byte,
                callback = self.update_bit_status,
                callback_args = {'index': j-1 },
                queue_size = 1,
            )
            self.topic_from1.append(topic_from1_)
            continue
        pass


    def update_bit_status(self, status, args):
        index = args["index"]
        self.bit_status[index] = status.data
        return

    def publish_status(self):
        bit_status_last = self.bit_status.copy()
        
        while not rospy.is_shutdown():
            if self.bit_status != bit_status_last:
                if self.bit_status == [0,1]: 
                    self.topic_to.publish("IN")
                elif self.bit_status == [1,0]:
                    self.topic_to.publish("OUT")
                elif self.bit_status == [1,1]:
                    self.topic_to.publish("MOVE")
                else:
                    pass

                bit_status_last = self.bit_status.copy()
                pass

            time.sleep(0.05)
            continue
        return

    def hot_position_cmd(self, command):
        lock = topic_utils.recv(name +"_lock", std_msgs.msg.Bool).data
    
        if lock == False:
            if command.data.upper() == "IN":
                for i in range(0,8):
                    if i == 4:
                        self.topic_to1[i].publish(1)
                    else:
                        self.topic_to1[i].publish(0)
            elif command.data.upper() == "OUT":
                for i in range(0,8):
                    if i == 1 or i == 4 or i ==5:
                        self.topic_to1[i].publish(1)
                    else:
                        self.topic_to1[i].publish(0)
            else:
                print("Command Error")
        return


if __name__ == "__main__":
    rospy.init_node(name)
    hot = hot_position()

    pub_thread = threading.Thread(
        target = hot.publish_status,
        daemon = True,
    )
    pub_thread.start()
    
    rospy.spin()
