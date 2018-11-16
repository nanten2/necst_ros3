#!/usr/bin/env python3

name = "dome_az_simulator"

import time
import threading
import rospy
import std_msgs.msg


class dome_az_simulator(object):
    
    command_speed = 0.0
    dome_az = 0.0

    status = [0,0,0,0]
    
    def __init__(self):

        self.topic_to = rospy.Publisher(
                name = "/cpz6204_rsw1/ch1",
                data_class = std_msgs.msg.Int64,
                queue_size = 1,
            )

        self.topic_from1 = rospy.Subscriber(
                name = "/necctrl/cpz2724_rsw2/do01",
                data_class = std_msgs.msg.Bool,
                callback = self.dome_az_simulator,
                callback_args = 0,
                queue_size = 1,
            )

        self.topic_from2 = rospy.Subscriber(
                name = "/necctrl/cpz2724_rsw2/do02",
                data_class = std_msgs.msg.Bool,
                callback = self.dome_az_simulator,
                callback_args = 1,
                queue_size = 1,
            )

        self.topic_from3 = rospy.Subscriber(
                name = "/necctrl/cpz2724_rsw2/do03",
                data_class = std_msgs.msg.Bool,
                callback = self.dome_az_simulator,
                callback_args = 2,
                queue_size = 1,
            )

        self.topic_from4 = rospy.Subscriber(
                name = "/necctrl/cpz2724_rsw2/do04",
                data_class = std_msgs.msg.Bool,
                callback = self.dome_az_simulator,
                callback_args = 3,
                queue_size = 1,
            )
        
        pass

    def dome_az_simulator(self, status, args):
        self.status[args] = status.data

        if self.status[1] == False:
            self.command_speed = 0

        else:
            if self.status[0] == False:
                if self.status[2] == True:
                    self.command_speed = 15
                elif self.status[3] == True:
                    self.command_speed = 30
                else:
                    self.command_speed = 3
            else:
                if self.status[2] == True:
                    self.command_speed = -15
                elif self.status[3] == True:
                    self.command_speed = -30
                else:
                    self.command_speed = -3

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
