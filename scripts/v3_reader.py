#!/usr/bin/env python3

name = 'v3_reader'

# ----
import time

import rospy
import std_msgs.msg
import necst.msg


class reader(object):

    def __init__(self, node=True):
        if node: rospy.init_node(name)
        else: pass

        self.ps = PS()

        # ----
        self.antenna = ANTENNA()
        self.dome = DOME()
        self.hot = HOT()
        self.m2 = M2()
        self.achilles = ACHILLES()
        self.weather = WEATHER()
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
        #TODO get rid WARN
        #self.sub[topic_name].unregister()
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


class ANTENNA(object):

    def __init__(self):
        self.ps = PS()
        pass

    def drive(self):
        name = "/antenna/drive"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def az(self):
        name = "/antenna/az"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def el(self):
        name = "/antenna/el"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

class DOME(object):

    def __init__(self):
        self.ps = PS()
        pass

    def az(self):
        name = "/dome/az"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def door(self):
        name = "/dome/door"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def memb(self):
        name = "/dome/memb"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

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

class M2(object):

    def __init__(self):
        self.ps = PS()
        pass

    def position(self):
        name = "/m2/position"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

class ACHILLES(object):

    def __init__(self):
        self.ps = PS()
        pass

    def oneshot_dfs1(self):
        name = "/achilles/data1"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float64MultiArray,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def oneshot_dfs2(self):
        name = "/achilles/data2"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float64MultiArray,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

class WEATHER(object):

    def __init__(self):
        self.ps = PS()
        pass

    def cabin_temp(self):
        name = "/weather/cabin_temp1"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def out_temp(self):
        name = "/weather/outside2_temp"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def out_humi(self):
        name = "/weather/outside2_humi"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def pressure(self):
        name = "/weather/press"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def wind_speed(self):
        name = "/weather/wind_speed"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret

    def wind_direction(self):
        name = "/weather/wind_direction"

        self.ps.set_subscriber(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
            )

        ret = self.ps.subscribe(topic_name=name)
        return ret
