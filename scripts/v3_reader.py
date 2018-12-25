#!/usr/bin/env python3

name = 'v3_reader'

# ----
import time

import rospy
import std_msgs.msg
import necst.msg


class reader(object):

    def __init__(self):
        rospy.init_node(name)
        self.ps = PS()

        # ----
        self.hot = HOT()
        pass

    def display_subscriber(self):
        [print(k) for k in self.ps.sub]
        return


class PS(object):
    sub = {
            #"topic_name":rospy.Subscriber(name, data_class, queue_size)
            }
    values = {
                #"topic_name": msg.data
                }
    
    def __init__(self):
        pass

    def subscribe(self, topic_name):
        self.sub[topic_name].unregister()
        return self.values[topic_name]

    def set_subscriber(self, topic_name, data_class, queue_size):
        self.sub[topic_name] = rospy.Subscriber(
                                        name = topic_name,
                                        data_class = data_class,
                                        callback = self.callback,
                                        callback_args = topic_name,
                                        queue_size = queue_size,
                                    )
        time.sleep(0.1)
        return

    def callback(self, msg, args):
        self.values[args] = msg.data
        return


class HOT(object):

    def __init__(self):
        self.ps = PS()
        pass

    def position(self):
        name = "/hot/position"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret
