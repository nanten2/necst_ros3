#! /usr/bin/env python3

name = "weather_ondotori_simulator"

# ----
import time
import random

import rospy
import std_msgs.msg

class weather_ondotori_sim(object):
    
    def __init__(self):

        self.pub_B_temp = rospy.Publisher(
            name = '/ondotori_B_temp',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_B_humi = rospy.Publisher(
            name = '/ondotori_B_humi',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_out_temp = rospy.Publisher(
            name = '/ondotori_outside_temp',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_out_humi = rospy.Publisher(
            name = '/ondotori_outside_humi',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_cabin1 = rospy.Publisher(
            name = '/ondotori_cabin_temp1',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_cabin2 = rospy.Publisher(
            name = '/ondotori_cabin_temp2',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_dome1 = rospy.Publisher(
            name = '/ondotori_dome_temp1',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_dome2 = rospy.Publisher(
            name = '/ondotori_dome_temp2',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        pass


    def pub_B(self):
        temp = 10 + random.uniform(-10, 10)
        humi = 15 + random.uniform(-10, 10)
        self.pub_B_temp.publish(temp)
        self.pub_B_humi.publish(humi)
        return

    def pub_out(self):
        temp = 10 + random.uniform(-10, 10)
        humi = 10 + random.uniform(-10, 10)
        self.pub_out_temp.publish(temp)
        self.pub_out_humi.publish(humi)
        return

    def pub_cabin(self):
        temp1 = 10 + random.uniform(-10, 10)
        temp2 = 10 + random.uniform(-10, 10)
        self.pub_cabin1.publish(temp1)
        self.pub_cabin2.publish(temp2)
        return

    def pub_dome(self):
        temp1 = 10 + random.uniform(-10, 10)
        temp2 = 10 + random.uniform(-10, 10)
        self.pub_dome1.publish(temp1)
        self.pub_dome2.publish(temp2)
        return


if __name__=="__main__":
    rospy.init_node(name)
    o = weather_ondotori_sim()
    
    while not rospy.is_shutdown():
        o.pub_B()
        o.pub_out()
        o.pub_cabin()
        o.pub_dome()

        time.sleep(5)
        continue
