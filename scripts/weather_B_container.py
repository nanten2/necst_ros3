#!/usr/bin/env python3

name = "weather_B_container"

import time
import threading
import rospy
import std_msgs.msg

class Weather_B_container(object):
    status = {
            "temp": 0.0,
            "humi": 0.0,
            }

    def __init__(self):
        
        self.topic_to1 = rospy.Publisher(
            name = name + "_temp",
            data_class = std_msgs.msg.Float64,
            latch = True,
            queue_size = 1,
        )
        
        self.topic_to2 = rospy.Publisher(
            name = name + "_humi",
            data_class = std_msgs.msg.Float64,
            latch = True,
            queue_size = 1,
        )

        self.topic_from1 = rospy.Subscriber(
            name = "ondotori_B_container_temp",
            data_class = std_msgs.msg.Float64,
            callback = self.weather_B_container_temp,
            queue_size = 1,
        )

        self.topic_from2 = rospy.Subscriber(
            name = "ondotori_B_container_humi",
            data_class = std_msgs.msg.Float64,
            callback = self.weather_B_container_humi,
            queue_size = 1,
        )
        pass

    def weather_B_container_temp(self, status):
        self.status["temp"] = status.data
        return
    
    def weather_B_container_humi(self, status):
        self.status["humi"] = status.data
        return

    def publish_status(self):
        status_last = self.status

        while not rospy.is_shutdown():
            if self.status != status_last:
                self.topic_to1.publish(status["temp"])
                self.topic_to2.publish(status["humi"])

                status_last = self.status.copy()
                pass

            time.sleep(0.1)
            continue
        return
    
if __name__=="__main__":
    rospy.init_node(name)
    weather_B_container = Weather_B_container()

    pub_thread = threading.Thread(
        target = weather_B_container.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
