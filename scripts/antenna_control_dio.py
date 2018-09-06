#!/usr/bin/env python3

name = "antenna_control_dio"

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
            name = "cpz2724_rsw0_di26",
            data_class = std_msgs.msg.Bool,
            latch = True
            queue_size = 1,
        )
    
    control_last = None
    while True:
        ret = di.input_point(26, 1)
        control = ret
        
        if control != control_last:
            topic_to.publish(control)
            control_last = control
        
        time.sleep(0.1)
        continue

