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

args = sys.argv
if len(args) != 2:
    print("INPUT RSW_ID")
    sys.exit(-1)


class logger(object):
    ch_list = ["01","02","03","04","05","06","07","08","09","10",
                "11","12","13","14","15","16","17","18","19","20",
                "21","22","23","24","25","26","27","28","29","30",
                "31","32"]
    
    def __init__(self):
        self.data_do = [0] * 32
        self.data_di = [0] * 32
        pass

    def make_table(self):
        self.n2.make_table("datatime", "(time float)")
        self.n2.make_table("data_do", "(do1,do2,do3,do4,do5,do6,do7,do8,do9,do10,do11,do12,do13,do14,do15,do16,do17,do18,do19,do20,do21,do22,do23,do24,do25,do26,do27,do28,do29,do30,do31,do32)")
        self.n2.make_table("data_di", "(di1,di2,di3,di4,di5,di6,di7,di8,di9,di10,di11,di12,di13,di14,di15,di16,di17,di18,di19,di20,di21,di22,di23,di24,di25,di26,di27,di28,di29,di30,di31,di32)")
        return

    def callback_do(self, req, args):
        self.data_do[int(args)-1] = req.data
        return

    def callback_di(self, req, args):
        self.data_di[int(args)-1] = req.data
        return

    def log(self):
        print("DATABASE OPEN")
        t = datetime.datetime.fromtimestamp(time.time())
        dbpath = '/home/amigos/data/cpz2724_rsw{}_logger/{}.db'.format(args[1], t.strftime('%Y%m%d_%H%M%S'))
        self.n2 = n2lite.N2lite(dbpath)
        self.make_table()
        while not rospy.is_shutdown():

            self.n2.write("datatime", "", (time.time(),))
            self.n2.write("data_do", "", tuple(self.data_do))
            self.n2.write("data_di", "", tuple(self.data_di))

            time.sleep(0.0001) # 0.1 msec.
        
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

    sub_do = [rospy.Subscriber(
                    name = "cpz2724_rsw{0}/do{1}".format(args[1], ch),
                    data_class = std_msgs.msg.Bool,
                    callback = st.callback_do,
                    callback_args = ch,
                    queue_size = 1
                ) for ch in st.ch_list]

    sub_di = [rospy.Subscriber(
                    name = "cpz2724_rsw{0}/di{1}".format(args[1], ch),
                    data_class = std_msgs.msg.Bool,
                    callback = st.callback_di,
                    callback_args = ch,
                    queue_size = 1
                ) for ch in st.ch_list]

    rospy.spin()
