#!/usr/bin/env python3

name = "weather_rain"

import threading
import rospy
import std_msgs.msg

class Weather_rain(object):
    status = {
            "rain": 0.0,
            }

    def __init__(self):
        
        self.topic_to1 = rospy.Publisher(
            name = name,
            data_class = std_msgs.msg.Float64,
            latch = True,
            queue_size = 1,
        )
        

        self.topic_from1 = rospy.Subscriber(
            name = "davis_rain",
            data_class = std_msgs.msg.Float64,
            callback = self.weather_rain,
            queue_size = 1,
        )
        pass

    def weather_rain(self, status):
        self.status["rain"] = status.data
        return

    def publish_status(self):
        status_last = self.status

        while not rospy.is_shutdown():
            if self.status != status_last:
                self.topic_to1.publish(status["rain"])

                status_last = self.status.copy()
                pass

            time.sleep(0.1)
            continue
        return
    
if __name__=="__main__":
    rospy.init_node(name)
    weather_rain = Weather_rain()

    pub_thread = threading.Thread(
        target = weather_rain.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
