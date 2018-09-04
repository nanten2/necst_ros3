#!/usr/bin/env python3

name = "antenna_drive_dio"

import time
import threading
import pyinterface
import rospy
import std_msgs.msg


board_name = 2724
output_rsw_id = 1
input_rsw_id = 0


def antenna_drive_do(command):
    if len(command.data) == 6:
        do.output_point(command.data[0:2], 1)
        do.output_point(command.data[2:6], 9)
    else:
        print("Command Error")
    return

def publish_drive():
    byte_last = []
    while not rospy.is_shutdown():
        byte = di.input_byte("IN1_4").to_list()

        if byte != byte_last:
            topic_to.publish(byte)
            byte_last = byte
            pass

        time.sleep(0.1)
        continue

    return


if __name__ == "__main__":
    rospy.init_node(name)
    
    do = pyinterface.open(board_name, output_rsw_id)
    di = pyinterface.open(board_name, input_rsw_id)
    
    topic_to = rospy.Publisher(
            name = "cpz2724_rsw0_di1_4",
            data_class = std_msgs.msg.ByteMultiArray,
            queue_size = 1,
            latch = True
        )

    topic_from = rospy.Subscriber(
            name = "cpz2724_rsw1_do1_9",
            data_class = std_msgs.msg.ByteMultiArray,
            callback = antenna_drive_do,
            queue_size = 1,
        )

    pub_thread = threading.Thread(
            target = publish_drive(),
            daemon = True
        )
    pub_thread.start()
    
    rospy.spin()
