#!/usr/bin/env python3

name = "antenna_drive"

import rospy
import std_msgs.msg


lock = False

def antenna_drive_mapper(status):
    if status.data[0:2] == [1,1] and status.data[2:4] == [1,1]:
        topic_to1.publish("on")
    elif status.data[0:2] == [0,0] and status.data[2:4] == [0,0]:
        topic_to1.publish("off")
    else:
        print("Error")
        pass
    return

def antenna_drive_cmd(command):
    if lock == True:
        return
    else:
        if command.data.lower() == "on":
            topic_to2.publish([1,1,1,1,1,1])
        elif command.data.lower() == "off":
            topic_to2.publish([0,0,0,0,0,0])
        else:
            print("Command Error")
    return

def antenna_drive_lock(status):
    global lock
    lock = status.data
    return


if __name__ == "__main__":
    rospy.init_node(name)
    
    topic_to1 = rospy.Publisher(
            name = name,
            data_class = std_msgs.msg.String,
            queue_size = 1,
            latch = True
        )

    topic_to2 = rospy.Publisher(
            name = "cpz2724_rsw1_do1_9",
            data_class = std_msgs.msg.ByteMultiArray,
            queue_size = 1,
            latch = True
        )

    topic_from1 = rospy.Subscriber(
            name = "cpz2724_rsw0_di1_4",
            data_class = std_msgs.msg.ByteMultiArray,
            callback = antenna_drive_mapper,
            queue_size = 1,
        )

    topic_from2 = rospy.Subscriber(
            name = name + "_cmd",
            data_class = std_msgs.msg.String,
            callback = antenna_drive_cmd,
            queue_size = 1,
        )

    topic_lock = rospy.Subscriber(
            name = name + "_lock",
            data_class = std_msgs.msg.Bool,
            callback = antenna_drive_lock,
            queue_size = 1,
        )

    rospy.spin()
