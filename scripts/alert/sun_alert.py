#!/usr/bin/env python3

name = "sun_alert"

import sys
import time
import threading
from datetime import datetime as dt
from datetime import timedelta
import astropy.units as u
from astropy.coordinates import SkyCoord, EarthLocation, AltAz, get_body
from astropy.time import Time    
import rospy
import std_msgs.msg

from necst.msg import List_coord_msg


class alert(object):

    az_list = ""
    el_list = ""

    emergency = ""
    warning = ""
    status_dome = ""
    status_memb = ""

    az = 0.
    el = 45.

    def __init__(self):

        self.topic_to = rospy.Publisher(
                name = "sun",
                data_class = std_msgs.msg.Bool,
                queue_size = 1,
            )

        self.stop = rospy.Publisher(
                name = "/obs/stop_cmd",
                data_class = std_msgs.msg.Bool,
                queue_size = 1,
            )

        sub_az = rospy.Subscriber(
                name = "/antenna/az",
                data_class = std_msgs.msg.Float64,
                callback = self.callback_az,
                queue_size = 1,
            )

        sub_el = rospy.Subscriber(
                name = "/antenna/el",
                data_class = std_msgs.msg.Float64,
                callback = self.callback_el,
                queue_size = 1,
            )

        sub_door = rospy.Subscriber(
                name = "/dome/door",
                data_class = std_msgs.msg.String,
                callback = self.callback_door,
                queue_size = 1,
            )

        sub_memb = rospy.Subscriber(
                name = "/dome/memb",
                data_class = std_msgs.msg.String,
                callback = self.callback_memb,
                queue_size = 1,
            )

        self.nanten2 = EarthLocation(lat = -22.96995611*u.deg, lon = -67.70308139*u.deg, height = 4863.85*u.m)
        pass
           
    def callback_az(self, req):
        self.az = req.data
        return
           
    def callback_el(self, req):
        self.el = req.data
        return

    def callback_door(self, req):
        self.status_dome = req.data
        return

    def callback_memb(self, req):
        self.status_memb = req.data
        return

    def thread_start(self):
        self.thread_alert = threading.Thread(target=self.pub_status)
        self.thread_alert.setDaemon(True)
        self.thread_alert.start()
        self.thread_sun = threading.Thread(target=self.check_sun_position)
        self.thread_sun.setDaemon(True)
        self.thread_sun.start()
        return

    def pub_status(self):
        while not rospy.is_shutdown():
            if self.emergency:
                print(self.emergency)
                self.topic_to.publish(True)
                self.stop.publish(True)
                self.emergency = ""
            elif self.warning:
                print(self.warning)
                self.topic_to.publish(False)
                self.warning = ""
            else:
                self.topic_to.publish(False)
            time.sleep(0.5)
        return

    def check_sun_position(self):
        """
        Emergency
        ---------
        |sun_az - real_az| < 15 [deg]
        |sun_el - real_el| < 15 [deg]

        """
        while not rospy.is_shutdown():
            warning = ""
            emergency = ""
            check_az = False
            check_el = False
            now = dt.utcnow()
            sun = get_body("sun", Time(now))#+timedelta(hours=12)))
            sun.location = self.nanten2
            azel = sun.altaz
            sun_az = azel.az.arcsec
            sun_el = azel.alt.arcsec
            az = self.az*3600
            el = self.el*3600

            if 0<abs(az-sun_az)<15*3600. or 345*3600.<abs(az-sun_az)<360*3600.:
                check_az = True
            if 0<abs(el-sun_el)<15*3600.:
                check_el = True

            if check_az and check_el and self.status_dome != "CLOSE" and self.status_memb != "CLOSE":
                emergency += "Emergency : antenna position near sun!! \n"
            elif check_az and check_el:
                warning += "Warning : antenna position near sun!! \n"
                warning += "dome status : "
                warning += self.status_dome+"\n"
                warning += "memb status : "
                warning += self.status_memb+"\n"             
            else:
                pass
            
            if emergency:
                self.emergency = emergency
            elif warning:
                self.warning = warning
            else:
                pass
            time.sleep(1.)
        return
    
if __name__ == "__main__":
    rospy.init_node(name)
    al = alert()
    al.thread_start()
    rospy.spin()
