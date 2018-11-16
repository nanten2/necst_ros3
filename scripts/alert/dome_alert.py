#!/usr/bin/env python3

name = 'dome_alert'

# ----
import time

import rospy
import std_msgs.msg


class dome_alert(object):

    def __init__(self):

        sub_emergency = rospy.Subscriber(
                name = "/dome/emergency",
                data_class = std_msgs.msg.Bool,
                callback = self.callback_emergency,
                queue_size = 1,
            )

        sub_error = rospy.Subscriber(
                name = "/dome/error",
                data_class = std_msgs.msg.Bool,
                callback = self.callback_error,
                queue_size = 1,
            )

        pass


    def callback_emergency(self, req):
        if req.data:
            print("[DOME] : !!! EMERGENCY !!! \n")
        else: pass
        return

    def callback_error(self, req):
        if req.data:
            print("[DOME] : ERROR {} \n".format(req.data))
        else: pass
        return


if __name__ == "__main__":
    rospy.init_node(name)
    alert = dome_alert()
    rospy.spin()

