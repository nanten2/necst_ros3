#!/usr/bin/env python3

name = "antenna_drive"

import time
import threading
import rospy
import std_msgs.msg
import topic_utils


class antenna_drive(object):
    bit_status = [0,0,0,0]

    def __init__(self):
        self.topic_to = rospy.Publisher(
                name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
                latch = True
            )

        self.topic_to1 = []
        for i in range(1,3):
            topic_to1_ = rospy.Publisher(
                name = "cpz2724_rsw1_do%d"%(i),
                data_class = std_msgs.msg.Byte,
                queue_size = 1,
                latch = True
            )
            self.topic_to1.append(topic_to1_)
            continue

        for j in range(9,13):
            topic_to1_ = rospy.Publisher(
                name = "cpz2724_rsw1_do%d"%(j),
                data_class = std_msgs.msg.Byte,
                queue_size = 1,
                latch = True
            )
            self.topic_to1.append(topic_to1_)
            continue

        self.topic_from = rospy.Subscriber(
                name = name + "_cmd",
                data_class = std_msgs.msg.String,
                callback = self.antenna_drive_cmd,
                queue_size = 1,
            )

        self.topic_from1 = []
        for k in range(1,5):
            topic_from1_ = rospy.Subscriber(
                name = 'cpz2724_rsw0_di%d'%(k),
                data_class = std_msgs.msg.Byte,
                callback = self.update_bit_status,
                callback_args = {'index': k-1 },
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
                if self.bit_status == [1,1,1,1]: 
                    self.topic_to.publish("on")
                elif self.bit_status == [0,0,0,0]:
                    self.topic_to.publish("off")
                else:
                    pass

                bit_status_last = self.bit_status.copy()
                pass

            time.sleep(0.05)
            continue
        return

    def antenna_drive_cmd(self, command):
        lock = topic_utils.recv(name +"_lock", std_msgs.msg.Bool).data
    
        if lock == False:
            if command.data.lower() == "on":
                for i in range(0,6):
                    self.topic_to1[i].publish(1)
            elif command.data.lower() == "off":
                for i in range(0,6):
                    self.topic_to1[i].publish(0)
            else:
                print("Command Error")
        return


if __name__ == "__main__":
    rospy.init_node(name)
    drive = antenna_drive()

    pub_thread = threading.Thread(
        target = drive.publish_status,
        daemon = True,
    )
    pub_thread.start()
    
    rospy.spin()
