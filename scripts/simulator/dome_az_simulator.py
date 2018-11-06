#!/usr/bin/env python3

name = "dome_az_simulator"

import time
import threading
import rospy
import std_msgs.msg


class dome_az_simulator(object):
    
    command_speed = 0.0
    dome_az = 0.0

    def __init__(self):

        self.topic_to = rospy.Publisher(
                name = "/cpz6204_rsw1/ax1",
                data_class = std_msgs.msg.Int32,
                latch = True,
                queue_size = 1,
            )

        self.topic_from = rospy.Subscriber(
                name = "az_speed_cmd",
                data_class = std_msgs.msg.String,
                callback = self.dome_az_simulator,
                queue_size = 1,
            )

    def dome_az_simulator(self, status):
        if status.data == "STOP":
            self.command_speed = 0
        if status.data == "<":
            self.command_speed = -1
        if status.data == "<<":
            self.command_speed = -5
        if status.data == "<<<":
            self.command_speed = -10
        if status.data == ">":
            self.command_speed = 1
        if status.data == ">>":
            self.command_speed = 5
        if status.data == ">>>":
            self.command_speed = 10
        else :pass
        return

    def publish_status(self):

        dome_az_last = None
        while not rospy.is_shutdown():
            self.dome_az += self.command_speed * 0.01

            if self.dome_az != dome_az_last:
                self.topic_to.publish(int(-1*(self.dome_az + 0.7238) * (2343/360)))

                dome_az_last = self.dome_az

            time.sleep(0.01)
            continue
        return

if __name__ == "__main__":
    rospy.init_node(name)
    dome_az_simulator = dome_az_simulator()
    pub_thread = threading.Thread(
        target = dome_az_simulator.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
