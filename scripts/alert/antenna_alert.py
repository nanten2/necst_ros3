#!/usr/bin/env python3

name = 'antenna_alert'

# ----
import time

import rospy
import std_msgs.msg

WARN = "\033[31m"
END = "\033[0m\n"

class antenna_alert(object):

    def __init__(self):

        sub_softlimit_az = rospy.Subscriber(
                name = "/antenna/az_soft_limit",
                data_class = std_msgs.msg.Bool,
                callback = self.callback_soft_az,
                queue_size = 1,
            )

        sub_softlimit_el = rospy.Subscriber(
                name = "/antenna/el_soft_limit",
                data_class = std_msgs.msg.Bool,
                callback = self.callback_soft_el,
                queue_size = 1,
            )

        sub_emergency = rospy.Subscriber(
                name = "/antenna/emergency",
                data_class = std_msgs.msg.Bool,
                callback = self.callback_emergency,
                queue_size = 1,
            )

        sub_error = rospy.Subscriber(
                name = "/antenna/error",
                data_class = std_msgs.msg.String,
                callback = self.callback_error,
                queue_size = 1,
            )

        pass

    def callback_soft_az(self, req):
        if req.data:
            print("[ANTENNA] : AZ SOFT LIMIT \n")
        else: pass
        return

    def callback_soft_el(self, req):
        if req.data:
            print("[ANTENNA] : EL SOFT LIMIT \n")
        else: pass
        return

    def callback_emergency(self, req):
        if req.data:
            print(WARN + "[ANTENNA] : !!! EMERGENCY !!! " + END)
        else: pass
        return

    def callback_error(self, req):
        if req.data:
            print(WARN + "[ANTENNA] : ERROR {} ".format(req.data) + END)
        else: pass
        return

if __name__ == "__main__":
    rospy.init_node(name)
    alert = antenna_alert()
    rospy.spin()


