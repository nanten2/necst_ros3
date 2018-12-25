#! /usr/bin/env python3

name = 'alert_monitor'

# ----
import time
import threading
import rospy
import std_msgs.msg

WARN = "\033[31m"
END = "\033[0m\n"

class alert_monitor(object):
    values = {
            #"name":normal,
            "/antenna/az_soft_limit":False,
            "/antenna/el_soft_limit":False,
            "/antenna/emergency":False,
            "/antenna/error":"",
            "/dome/error":"",
            #"name":[threashold],
            "/weather/rain":[0.5, 1.0],
            "/weather/outside2_humi":[60, 80],
            "/weather/wind_speed":[10, 15],
            }
    
    def __init__(self):
        def new(name, data_class):
            rospy.Subscriber(
                name = name,
                data_class = data_class,
                callback = self.callback,
                callback_args = {'name': name},
                queue_size = 1,
            )
            return            

        # Antenna
        # -------
        new('/antenna/az_soft_limit', std_msgs.msg.Bool)
        new('/antenna/el_soft_limit', std_msgs.msg.Bool)
        new('/antenna/emergency', std_msgs.msg.Bool)
        new('/antenna/error', std_msgs.msg.String)

        # Dome
        # -------
        new('/dome/error', std_msgs.msg.String)

        # Weather
        # -------
        new('/weather/rain', std_msgs.msg.Float32)
        new('/weather/outside2_humi', std_msgs.msg.Float32)
        new('/weather/wind_speed', std_msgs.msg.Float32)

        pass
    

    def callback(self, msg, args):
        if type(msg.data) == float:
            if msg.data >= self.values[args['name']][1]:
                print(WARN + "[ALERT] : {0} over {1}".format(args['name'].upper(), self.values[args['name']][1]) + END)
            elif msg.data > self.values[args['name']][0]:
                print("[ALERT] : {0} over {1}".format(args['name'].upper(), self.values[args['name']][0]))
            else: pass

        else:
            if msg.data != self.values[args['name']]:
                print(WARN + "[ALERT] : {0}, {1}".format(args['name'].upper(), msg.data) + END)
            else: pass
        return


if __name__=='__main__':
    rospy.init_node(name)
    am = alert_monitor()
    rospy.spin()
