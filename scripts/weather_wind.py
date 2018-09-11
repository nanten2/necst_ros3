#!/usr/bin/env python3

name = "weather_wind"

import threading
import rospy
import std_msgs.msg

class Weather_wind(object):
    status = {
            "speed": 0.0,
            "direction": 0.0,
            }

    def __init__(self):
        
        self.topic_to1 = rospy.Publisher(
            name = name + "_speed",
            data_class = std_msgs.msg.Float64,
            latch = True,
            queue_size = 1,
        )
        
        self.topic_to2 = rospy.Publisher(
            name = name + "_direction",
            data_class = std_msgs.msg.Float64,
            latch = True,
            queue_size = 1,
        )

        self.topic_from1 = rospy.Subscriber(
            name = "davis_wind_speed",
            data_class = std_msgs.msg.Float64,
            callback = self.weather_wind_speed,
            queue_size = 1,
        )

        self.topic_from2 = rospy.Subscriber(
            name = "davis_wind_direction",
            data_class = std_msgs.msg.Float64,
            callback = self.weather_wind_direction,
            queue_size = 1,
        )
        pass

    def weather_wind_speed(self, status):
        self.status["speed"] = status.data
        return
    
    def weather_wind_direction(self, status):
        self.status["direction"] = status.data
        return

    def publish_status(self):
        status_last = self.status

        while not rospy.is_shutdown():
            if self.status != status_last:
                self.topic_to1.publish(status["speed"])
                self.topic_to2.publish(status["direction"])

                status_last = self.status.copy()
                pass

            time.sleep(0.1)
            continue
        return
    
if __name__=="__main__":
    rospy.init_node(name)
    weather_wind = Weather_wind()

    pub_thread = threading.Thread(
        target = weather_wind.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
