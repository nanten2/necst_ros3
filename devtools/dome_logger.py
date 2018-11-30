#!/usr/bin/env python3

name = 'dome_logger'

import sys
import time
import datetime
import threading
sys.path.append("/home/amigos/python/")
from n2lite import n2lite

import rospy
import std_msgs.msg


class logger(object):

    def __init__(self):
        self.az = 0
        self.az_cmd = 0
        self.az_speed_cmd = 0
        self.az_action = ""
        self.az_switch = 0
        self.az_tracking = False

        self.door = ""
        self.door_cmd = ""
        self.door_cmd2 = ""
        self.door_right_pos = ""
        self.door_left_pos = ""
        self.door_right_act = ""
        self.door_left_act = ""

        self.memb = ""
        self.memb_cmd = ""
        self.memb_cmd2 = ""
        self.memb_act = ""

        self.error = ""
        self.control = ""

        pass

    def make_table(self):
        self.n2.make_table("datatime", "(time float)")
        self.n2.make_table("dome_az", "(az float, az_cmd float, az_speed_cmd float, az_action string, az_switch int, az_tracking)")
        self.n2.make_table("door", "(door string, door_cmd string, door_cmd2 string, door_right_pos string, door_left_pos string, door_right_act string, door_left_act string)")
        self.n2.make_table("memb", "(memb string, memb_cmd string, memb_cmd2 string, memb_act string)")
        self.n2.make_table("io", "(error string, control string)")
        return

    def callback_az(self, req):
        self.az = req.data
        return

    def callback_az_cmd(self, req):
        self.az_cmd = req.data
        return

    def callback_az_speed_cmd(self, req):
        self.az_speed_cmd = req.data
        return

    def callback_az_action(self, req):
        self.az_action = req.data
        return

    def callback_az_switch(self, req):
        self.az_switch = req.data
        return

    def callback_az_tracking(self, req):
        self.az_tracking = req.data
        return

    def callback_door(self, req):
        self.door = req.data
        return

    def callback_door_cmd(self, req):
        self.door_cmd = req.data
        return

    def callback_door_cmd2(self, req):
        self.door_cmd2 = req.data
        return

    def callback_door_right_pos(self, req):
        self.door_right_pos = req.data
        return

    def callback_door_left_pos(self, req):
        self.door_left_pos = req.data
        return

    def callback_door_right_act(self, req):
        self.door_right_act = req.data
        return

    def callback_door_left_act(self, req):
        self.door_left_act = req.data
        return

    def callback_memb(self, req):
        self.memb = req.data
        return

    def callback_memb_cmd(self, req):
        self.memb_cmd = req.data
        return

    def callback_memb_cmd2(self, req):
        self.memb_cmd2 = req.data
        return

    def callback_memb_act(self, req):
        self.memb_act = req.data
        return

    def callback_error(self, req):
        self.error = req.data
        return

    def callback_control(self, req):
        self.control = req.data
        return

    def log(self):
        print("DATABASE OPEN")
        t = datetime.datetime.fromtimestamp(time.time())
        dbpath = '/home/amigos/data/dome_logger/{}.db'.format(t.strftime('%Y%m%d_%H%M%S'))
        self.n2 = n2lite.N2lite(dbpath)
        self.make_table()
        while not rospy.is_shutdown():

            self.n2.write("datatime", "", (time.time(),))
            self.n2.write("dome_az", "", (self.az, self.az_cmd, self.az_speed_cmd, self.az_action, self.az_switch, self.az_tracking))
            self.n2.write("door", "", (self.door, self.door_cmd, self.door_cmd2, self.door_right_pos, self.door_left_pos, self.door_right_act, self.door_left_act))
            self.n2.write("memb", "", (self.memb, self.memb_cmd, self.memb_cmd2, self.memb_act))
            self.n2.write("io", "", (self.error, self.control))

            time.sleep(0.0001) # 0.1msec.
        
        else: 
            self.n2.commit_data()
            self.n2.close()
            print("DATABASE CLOSE")
        return

    def start_thread(self):
        th = threading.Thread(target=self.log)
        th.setDaemon(True)
        th.start()


if __name__ == '__main__':
    rospy.init_node(name)
    
    st = logger()
    st.start_thread()

    sub_az = rospy.Subscriber(
            name = "/dome/az",
            data_class = std_msgs.msg.Float32,
            callback = st.callback_az,
            queue_size = 1,
        )

    sub_az_cmd = rospy.Subscriber(
            name = "/dome/az_cmd",
            data_class = std_msgs.msg.Float32,
            callback = st.callback_az_cmd,
            queue_size = 1,
        )

    sub_az_speed_cmd = rospy.Subscriber(
            name = "/dome/az_speed_cmd",
            data_class = std_msgs.msg.String,
            callback = st.callback_az_speed_cmd,
            queue_size = 1,
        )

    sub_az_action = rospy.Subscriber(
            name = "/dome/az_action",
            data_class = std_msgs.msg.String,
            callback = st.callback_az_action,
            queue_size = 1,
        )

    sub_az_tracking = rospy.Subscriber(
            name = "/dome/az_tracking",
            data_class = std_msgs.msg.Bool,
            callback = st.callback_az_tracking,
            queue_size = 1,
        )

    sub_door = rospy.Subscriber(
            name = "/dome/door",
            data_class = std_msgs.msg.String,
            callback = st.callback_door,
            queue_size = 1,
        )

    sub_door_cmd = rospy.Subscriber(
            name = "/dome/door_cmd",
            data_class = std_msgs.msg.String,
            callback = st.callback_door_cmd,
            queue_size = 1,
        )

    sub_door_cmd2 = rospy.Subscriber(
            name = "/dome/door_cmd2",
            data_class = std_msgs.msg.String,
            callback = st.callback_door_cmd2,
            queue_size = 1,
        )

    sub_door_right_pos = rospy.Subscriber(
            name = "/dome/door_right_pos",
            data_class = std_msgs.msg.String,
            callback = st.callback_door_right_pos,
            queue_size = 1,
        )

    sub_door_left_pos = rospy.Subscriber(
            name = "/dome/door_left_pos",
            data_class = std_msgs.msg.String,
            callback = st.callback_door_left_pos,
            queue_size = 1,
        )

    sub_door_right_act = rospy.Subscriber(
            name = "/dome/door_right_act",
            data_class = std_msgs.msg.String,
            callback = st.callback_door_right_act,
            queue_size = 1,
        )

    sub_door_left_act = rospy.Subscriber(
            name = "/dome/door_left_act",
            data_class = std_msgs.msg.String,
            callback = st.callback_door_left_act,
            queue_size = 1,
        )

    sub_memb = rospy.Subscriber(
            name = "/dome/memb",
            data_class = std_msgs.msg.String,
            callback = st.callback_memb,
            queue_size = 1,
        )

    sub_memb_cmd = rospy.Subscriber(
            name = "/dome/memb_cmd",
            data_class = std_msgs.msg.String,
            callback = st.callback_memb_cmd,
            queue_size = 1,
        )

    sub_memb_cmd2 = rospy.Subscriber(
            name = "/dome/memb_cmd2",
            data_class = std_msgs.msg.String,
            callback = st.callback_memb_cmd2,
            queue_size = 1,
        )

    sub_memb_act = rospy.Subscriber(
            name = "/dome/memb_act",
            data_class = std_msgs.msg.String,
            callback = st.callback_memb_act,
            queue_size = 1,
        )

    sub_error = rospy.Subscriber(
            name = "/dome/error",
            data_class = std_msgs.msg.String,
            callback = st.callback_error,
            queue_size = 1,
        )

    sub_control = rospy.Subscriber(
            name = "/dome/control",
            data_class = std_msgs.msg.String,
            callback = st.callback_control,
            queue_size = 1,
        )

    rospy.spin()
