#!/usr/bin/env python3

name = "encoder_el_sim"

import time
import threading
import rospy
import std_msgs.msg


class encoder_el_sim(object):
    
    command_speed = 0.0
    enc_el = 45.0 * 3600.

    def __init__(self):

        self.topic_to = rospy.Publisher(
                name = "/cpz6204_rsw0/ch02",
                data_class = std_msgs.msg.Int64,
                latch = True,
                queue_size = 1,
            )

        self.topic_from = rospy.Subscriber(
                name = "/antenna/el_speed",
                data_class = std_msgs.msg.Float64,
                callback = self.encoder_el_sim,
                queue_size = 1,
            )

    def encoder_el_sim(self, status):
        self.command_speed = status.data
        return

    def publish_status(self):

        enc_el_last = None
        while not rospy.is_shutdown():
            self.enc_el += self.command_speed * 0.01 * 0.617

            if self.enc_el != enc_el_last:
                self.topic_to.publish(int(self.enc_el / (360*3600/(23600*400))))

                enc_el_last = self.enc_el

            time.sleep(0.01)
            continue

if __name__ == "__main__":
    rospy.init_node(name)
    enc_el_sim = encoder_el_sim()
    pub_thread = threading.Thread(
        target = enc_el_sim.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
