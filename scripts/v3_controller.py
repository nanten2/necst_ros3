#!/usr/bin/env python3

name = 'v3_controller'

# ----
import time

import rospy
import std_msgs.msg
import necst.msg


class controller(object):

    def __init__(self):
        rospy.init_node(name)
        self.ps = PS()

        # ----
        self.antenna = ANTENNA()
        self.dome = DOME()
        self.m2 = M2()
        self.m4 = M4()
        self.hot = HOT()
        pass

    def display_publisher(self):
        print(self.ps.pub.keys())
        return

    def delete_publisher(self):
        self.ps.pub.clear()
        return

    """
    def display_subscriber(self):
        print(self.ps.sub.keys())
        return

    def delete_subscriber(self):
        self.ps.sub.clear()
        return
    """


class PS(object):
    pub = {
            #"topic_name":rospy.Publisher(name, data_class, queue_size, latch)
            }
    
    """
    sub = {
            #"topic_name":[rospy.Subscriber(name, data_class, callback, callback_args), 0]
            }
    """

    def __init__(self):
        """
        sub_th = threading.Thread(
                target = self.sub_function,
                daemon = True
                )
        sub_th.start()
        """
        pass

    def publish(self, topic_name, msg):
        self.pub[topic_name].publish(msg)
        return
    
    """
    def subscribe(self, topic_name):
        return self.sub[topic_name][1]

    def callback(self, req, topic_name):
        self.sub[topic_name][1] = req.data
        return
    """

    def set_publisher(self, topic_name, data_class, queue_size, latch=True):
        if topic_name not in self.pub:
            self.pub[topic_name] = rospy.Publisher(
                                            name = topic_name,
                                            data_class = data_class,
                                            queue_size = queue_size,
                                            latch = latch,
                                        )
            time.sleep(0.01)
        else: pass
        return

    """
    def set_subscriber(self, topic_name, data_class):
        self.sub[topic_name] = [rospy.Subscriber(
                                            name = topic_name,
                                            data_class = data_class,
                                            callback = self.callback,
                                            callback_args = topic_name
                                        ), None]
        return

    def sub_function(self):
        rospy.spin()
        return
    """


class ANTENNA(object):

    def __init__(self):
        self.ps = PS()
        pass

    def drive(self, command):
        name = "/antenna/drive_cmd"
        
        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=command)
        return

    def _az_move(self, command_az): # deg
        name_az = "/antenna/az_cmd"

        self.ps.set_publisher(
                topic_name = name_az,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
                latch = False
            )

        self.ps.publish(topic_name=name_az, msg=command_az)
        return

    def _el_move(self, command_el): # deg
        name_el = "/antenna/el_cmd"
        
        self.ps.set_publisher(
                topic_name = name_el,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
                latch = False
            )

        self.ps.publish(topic_name=name_el, msg=command_el)
        
        return

    def onepoint_move(self, command_az, command_el, coord="altaz", off_x=0, off_y=0, offcoord="altaz", hosei="hosei_230.txt", lamda=2600, dcos=0, limit=True, rotation=True):
        name = "/obs/onepoint_command"

        self.ps.set_publisher(
                topic_name = name,
                data_class = necst.msg.Move_mode_msg,
                queue_size = 1,
                latch = False
            )

        command = necst.msg.Move_mode_msg()
        command.x = command_az
        command.y = command_el
        command.coord = coord
        command.planet = planet
        command.off_x = off_x
        command.off_y = off_y
        command.offcoord = offcoord
        command.hosei = hosei
        command.lamda = lamda
        command.dcos = dcos
        command.limit = limit
        command.rotation = rotation
        command.timestamp = time.time()

        self.ps.publish(topic_name=name, msg=command)
        return


class DOME(object):

    def __init__(self):
        self.ps = PS()
        pass

    def move(self, dist): #deg
        name = "/dome/az_cmd"
        
        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=dist)
        return

    def door(self, command):
        name = "/dome/door_cmd"
        
        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=command)
        return

    def memb(self, command):
        name = "/dome/memb_cmd"
        
        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=command)
        return

class M2(object):

    def __init__(self):
        self.ps = PS()
        pass

    def move(self, dist):
        name = "/m2/position_cmd"
        
        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=dist)
        return

class M4(object):

    def __init__(self):
        self.ps = PS()
        pass

    def position(self, command):
        name = "/m4/position_cmd"
        
        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=command)
        return

class HOT(object):

    def __init__(self):
        self.ps = PS()
        pass

    def position(self, command):
        name = "/hot/position_cmd"
        
        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=command)
        return
