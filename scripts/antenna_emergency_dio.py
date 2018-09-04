#!/usr/bin/env python3

name = "antenna_emergency_dio"

import time
import pyinterface
import rospy
import std_msgs.msg


board_name = 2724
rsw_id = 0


if __name__ == "__main__":
    rospy.init_node(name)
    di = pyinterface.open(board_name, rsw_id)

    topic_to = rospy.Publisher(
            name = "cpz2724_rsw0_di24",
            data_class = std_msgs.msg.Bool,
            latch = True
            queue_size = 1,
        )
    
    emergency_last = None
    while True:
        ret = di.input_dword().to_list()
        emergency = ret[24]
        
        if emergency != emergency_last:
            topic_to.publish(emergency)
            emergency_last = emergency
        
        time.sleep(0.1)
        continue

