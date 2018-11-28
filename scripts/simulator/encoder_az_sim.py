#!/usr/bin/env python3

name = "encoder_az_sim"

import time
import threading
import rospy
import std_msgs.msg


class encoder_az_sim(object):
    
    command_speed = 0.0
    enc_az = 0.0

    def __init__(self):

        self.topic_to = rospy.Publisher(
                name = "/cpz6204_rsw0/ch01",
                data_class = std_msgs.msg.Int64,
                latch = True,
                queue_size = 1,
            )

        self.topic_from = rospy.Subscriber(
                name = "/antenna/az_speed",
                data_class = std_msgs.msg.Float64,
                callback = self.encoder_az_sim,
                queue_size = 1,
            )

    def encoder_az_sim(self, status):
        self.command_speed = status.data
        return

    def publish_status(self):

        enc_az_last = None
        while not rospy.is_shutdown():
            self.enc_az += self.command_speed * 0.1 * 0.1

            if self.enc_az != enc_az_last:
                self.topic_to.publish(int(self.enc_az / (360*3600/(23600*400))))

                enc_az_last = self.enc_az

            time.sleep(0.01)
            continue

if __name__ == "__main__":
    rospy.init_node(name)
    enc_az_sim = encoder_az_sim()
    pub_thread = threading.Thread(
        target = enc_az_sim.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
