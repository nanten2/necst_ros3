#!/usr/bin/env python3

name = "weather_dome"

import time
import threading
import rospy
import std_msgs.msg

class Weather_dome(object):
    status = {
            "temp1": 0.0,
            "temp2": 0.0,
            }

    def __init__(self):
        
        self.topic_to1 = rospy.Publisher(
            name = name + "_temp1",
            data_class = std_msgs.msg.Float64,
            latch = True,
            queue_size = 1,
        )
        
        self.topic_to2 = rospy.Publisher(
            name = name + "_temp2",
            data_class = std_msgs.msg.Float64,
            latch = True,
            queue_size = 1,
        )

        self.topic_from1 = rospy.Subscriber(
            name = "dome_temp1",
            data_class = std_msgs.msg.Float64,
            callback = self.weather_dome_temp1,
            queue_size = 1,
        )

        self.topic_from2 = rospy.Subscriber(
            name = "dome_temp2",
            data_class = std_msgs.msg.Float64,
            callback = self.weather_dome_temp2,
            queue_size = 1,
        )
        pass

    def weather_dome_temp1(self, status):
        self.status["temp1"] = status.data
        return
    
    def weather_dome_temp2(self, status):
        self.status["temp2"] = status.data
        return

    def publish_status(self):
        status_last = self.status

        while not rospy.is_shutdown():
            if self.status != status_last:
                self.topic_to1.publish(status["temp1"])
                self.topic_to2.publish(status["temp2"])

                status_last = self.status.copy()
                pass

            time.sleep(0.1)
            continue
        return
    
if __name__=="__main__":
    rospy.init_node(name)
    weather_dome = Weather_dome()

    pub_thread = threading.Thread(
        target = weather_dome.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
