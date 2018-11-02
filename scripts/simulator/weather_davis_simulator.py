#! /usr/bin/env python3

name = "weather_davis_simulator"

# ----
import time
import random

import rospy
import std_msgs.msg

class weather_davis_sim(object):
    
    def __init__(self):

        self.pub_D_temp = rospy.Publisher(
            name = '/davis_D_temp',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_D_humi = rospy.Publisher(
            name = '/davis_D_humi',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_out_temp = rospy.Publisher(
            name = '/davis_outside_temp',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_out_humi = rospy.Publisher(
            name = '/davis_outside_humi',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_press = rospy.Publisher(
            name = '/davis_press',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_rain = rospy.Publisher(
            name = '/davis_rain',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_wind_dir = rospy.Publisher(
            name = '/davis_wind_direction',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        self.pub_wind_sp = rospy.Publisher(
            name = '/davis_wind_speed',
            data_class = std_msgs.msg.Float32,
            latch = True,
            queue_size = 1,
        )

        pass


    def pub_D(self):
        temp = 10 + random.uniform(-10, 10)
        humi = 15 + random.uniform(-10, 10)
        self.pub_D_temp.publish(temp)
        self.pub_D_humi.publish(humi)
        return

    def pub_out(self):
        temp = 10 + random.uniform(-10, 10)
        humi = 10 + random.uniform(-10, 10)
        self.pub_out_temp.publish(temp)
        self.pub_out_humi.publish(humi)
        return

    def pub_pr(self):
        press = 900 + random.uniform(-10, 10)
        self.pub_press.publish(press)
        return

    def pub_rai(self):
        rain = 0.0
        self.pub_rain.publish(rain)
        return

    def pub_wind(self):
        speed = 5 + random.uniform(-5, 6)
        direc = 47 + random.uniform(-10,10)
        self.pub_wind_dir.publish(direc)
        self.pub_wind_sp.publish(speed)
        return


if __name__=="__main__":
    rospy.init_node(name)
    d = weather_davis_sim()
    
    while not rospy.is_shutdown():
        d.pub_D()
        d.pub_out()
        d.pub_pr()
        d.pub_rai()
        d.pub_wind()

        time.sleep(5)
        continue
