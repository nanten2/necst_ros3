#!/usr/bin/env python3

name = 'antenna_logger'

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
        self.el = 0
        self.az_cmd = 0
        self.el_cmd = 0
        self.az_cmd2 = 0
        self.el_cmd2 = 0
        self.az_speed = 0
        self.el_speed = 0
        pass

    def make_table(self):
        self.n2.make_table("datatime", "(time float)")
        self.n2.make_table("encoder", "(az float, el float)")
        self.n2.make_table("command", "(az_cmd float, el_cmd float)")
        self.n2.make_table("command2", "(az_cmd2 float, el_cmd2 float)")
        self.n2.make_table("speed", "(az_speed float, el_speed float)")
        return

    def callback_az(self, req):
        self.az = req.data
        return

    def callback_el(self, req):
        self.el = req.data
        return

    def callback_az_cmd(self, req):
        self.az_cmd = req.data
        return

    def callback_az_cmd2(self, req):
        self.az_cmd2 = req.data
        return

    def callback_el_cmd(self, req):
        self.el_cmd = req.data
        return

    def callback_el_cmd2(self, req):
        self.el_cmd2 = req.data
        return

    def callback_az_speed(self, req):
        self.az_speed = req.data
        return

    def callback_el_speed(self, req):
        self.el_speed = req.data
        return

    def log(self):
        print("DATABASE OPEN")
        t = datetime.datetime.fromtimestamp(time.time())
        dbpath = '/home/amigos/data/antenna_{}.db'.format(t.strftime('%Y%m%d_%H%M%S'))
        self.n2 = n2lite.N2lite(dbpath)
        self.make_table()
        while not rospy.is_shutdown():

            self.n2.write("datatime", "", (time.time(),))
            self.n2.write("encoder", "", (self.az, self.el))
            self.n2.write("command", "", (self.az_cmd, self.el_cmd))
            self.n2.write("command2", "", (self.az_cmd2, self.el_cmd2))
            self.n2.write("speed", "", (self.az_speed, self.el_speed))
            self.n2.commit_data()

            time.sleep(0.001) # 10 msec.
        
        else: 
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
            name = "/antenna/az",
            data_class = std_msgs.msg.Float64,
            callback = st.callback_az,
            queue_size = 1,
        )

    sub_el = rospy.Subscriber(
            name = "/antenna/el",
            data_class = std_msgs.msg.Float64,
            callback = st.callback_el,
            queue_size = 1,
        )

    sub_az_cmd = rospy.Subscriber(
            name = "/antenna/az_cmd",
            data_class = std_msgs.msg.Float64,
            callback = st.callback_az_cmd,
            queue_size = 1,
        )

    sub_el_cmd = rospy.Subscriber(
            name = "/antenna/el_cmd",
            data_class = std_msgs.msg.Float64,
            callback = st.callback_el_cmd,
            queue_size = 1,
        )

    sub_az_cmd2 = rospy.Subscriber(
            name = "/antenna/az_cmd2",
            data_class = std_msgs.msg.Float64,
            callback = st.callback_az_cmd2,
            queue_size = 1,
        )

    sub_el_cmd2 = rospy.Subscriber(
            name = "/antenna/el_cmd2",
            data_class = std_msgs.msg.Float64,
            callback = st.callback_el_cmd2,
            queue_size = 1,
        )

    sub_az_speed = rospy.Subscriber(
            name = "/antenna/az_speed",
            data_class = std_msgs.msg.Float64,
            callback = st.callback_az_speed,
            queue_size = 1,
        )

    sub_el_speed = rospy.Subscriber(
            name = "/antenna/el_speed",
            data_class = std_msgs.msg.Float64,
            callback = st.callback_el_speed,
            queue_size = 1,
        )

    rospy.spin()
