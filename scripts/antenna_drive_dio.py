#!/usr/bin/env python3

name = "antenna_drive_dio"

import time
import threading
import pyinterface
import rospy
import std_msgs.msg


class antenna_drive_dio(object):

    bit_status = [0,0,0,0,0,0]
    count = 0

    board_name = 2724
    output_rsw_id = 1
    input_rsw_id = 0

    def __init__(self):
        self.do = pyinterface.open(self.board_name, self.output_rsw_id)
        self.di = pyinterface.open(self.board_name, self.input_rsw_id)

        self.topic_to = []
        for i in range(1,5):
            topic_to_ = rospy.Publisher(
                    name = "cpz2724_rsw0_di%d"%(i),
                    data_class = std_msgs.msg.Byte,
                    queue_size = 1,
                    latch = True
                )
            self.topic_to.append(topic_to_)
            continue


        self.topic_from = []
        for j in range(1,3):
            topic_from_ = rospy.Subscriber(
                    name = 'cpz2724_rsw1_do%d'%(j),
                    data_class = std_msgs.msg.Byte,
                    callback = self.update_bit_status,
                    callback_args = {'index': j-1 },
                    queue_size = 1,
                )
            self.topic_from.append(topic_from_)
            continue

        for k in range(9,13):
            topic_from_ = rospy.Subscriber(
                    name = 'cpz2724_rsw1_do%d'%(k),
                    data_class = std_msgs.msg.Byte,
                    callback = self.update_bit_status,
                    callback_args = {'index': k-7 },
                    queue_size = 1,
                )
            self.topic_from.append(topic_from_)
            continue

        pass

    def update_bit_status(self, command, args):
        index = args["index"]
        self.bit_status[index] = command.data

        self.count += 1
        if self.count == 6:
            self.antenna_drive_do()

        return

    def antenna_drive_do(self):
        self.do.output_point(self.bit_status[0:2], 1)
        self.do.output_point(self.bit_status[2:6], 9)
        return

    def publish_drive(self):
        byte_last = []
        while not rospy.is_shutdown():
            byte = di.input_byte("IN1_4").to_list()

            if byte != byte_last:
                for i in range(0,4):
                    topic_to[i].publish(byte[i])

                byte_last = byte
            time.sleep(0.05)
            continue

        return


if __name__ == "__main__":
    rospy.init_node(name)
    drive_dio = antenna_drive_dio()
    
    pub_thread = threading.Thread(
            target = drive_dio.publish_drive(),
            daemon = True
        )
    pub_thread.start()
    
    rospy.spin()
