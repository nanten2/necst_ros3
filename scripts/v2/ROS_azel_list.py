#! /usr/bin/env python3

import rospy
import std_msgs.msg
from necst.msg import List_coord_msg

from datetime import datetime
from astropy.time import Time
import time
import sys
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/necst/ros/src/necst/lib/")
import calc_coord


node_name = "azel_list"


class azel_list(object):

    press = 500
    out_temp = 0
    out_humi = 0
    param = ""
    stop_flag = False
    old_list = ""

    def __init__(self):
        self.start_time = time.time()
        rospy.Subscriber("wc_list", List_coord_msg, self._receive_list, queue_size=1)
        rospy.Subscriber("stop_cmd", std_msgs.msg.Bool, self._stop, queue_size=1)
        rospy.Subscriber("/weather/press", std_msgs.msg.Float32, self._receive_press, queue_size=1)
        rospy.Subscriber("/weather/outside2_temp", std_msgs.msg.Float32, self._receive_temp, queue_size=1)
        rospy.Subscriber("/weather/outside2_humi", std_msgs.msg.Float32, self._receive_humi, queue_size=1)

        self.pub = rospy.Publisher("list_azel", List_coord_msg, queue_size=1000)
        
        self.calc = calc_coord.azel_calc()
        pass

    def _receive_press(self, req):
        if req.data == 0:
            req.data = 500
        self.press = req.data
        return

    def _receive_temp(self, req):
        if req.data > 273.15:
            req.data -= 273.15
        self.out_temp = req.data
        return

    def _receive_humi(self, req):
        if req.data > 1:
            req.data = req.data/100.
        self.out_humi = req.data
        return
    
    def _receive_list(self, req):
        ### x,y is [arcsec]
        if req.timestamp < self.start_time:
            print("receive_old_list...")
        else:
            self.stop_flag = False
            self.param = req
            pass
        return

    def _stop(self, req):
        if req.data:
            self.stop_flag = req.data
        else: pass
        return

    def create_azel_list(self):
        msg = List_coord_msg()
        print("wait comming list...")
        while (self.param =="") and (not rospy.is_shutdown()) :
            time.sleep(0.1)
            continue
        print("start_calclation!!")
        loop = 0
        check = 0
        param = self.param        
        while not rospy.is_shutdown():
            if not self.param:
                time.sleep(1.)
                continue
            elif param != self.param:
                loop = 0
                check = 0
                param = self.param
            else:
                pass
            
            if self.stop_flag == False:
                if len(param.x_list) > 2:
                    dt = 0.1                    

                    # linear fitting
                    len_x = param.x_list[loop+1] - param.x_list[loop]
                    len_y = param.y_list[loop+1] - param.y_list[loop]
                    len_t = param.time_list[loop+1] - param.time_list[loop]
                
                    dx = len_x/(len_t*10)#[arcsec/100ms]
                    dy = len_y/(len_t*10)#[arcsec/100ms]
                    dt = 0.1

                    x_list2 = [param.x_list[loop] + dx*(i+check*10) for i in range(10)]
                    y_list2 = [param.y_list[loop] + dy*(i+check*10) for i in range(10)]
                    time_list2 = [param.time_list[loop]+dt*(i+check*10) for i in range(10)]
                    loop_count = 0
                    check_count = 1
                    for i in range(10):
                        if param.time_list[-1]< time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            self.stop_flag = True
                        elif param.time_list[loop+1] <= time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            loop_count = 1
                            check = 0
                            check_count = 0
                        else:
                            break
                    loop += loop_count                        
                    if loop == len(param.time_list)-1:
                        self.stop_flag = True                        
                    check +=  check_count
                else:
                    len_x = param.x_list[1] - param.x_list[0]
                    len_y = param.y_list[1] - param.y_list[0]
                    len_t = param.time_list[1] - param.time_list[0]

                    dx = len_x/(len_t*10)#[arcsec/100ms]
                    dy = len_y/(len_t*10)#[arcsec/100ms]
                    dt = 0.1
                
                    x_list2 = [param.x_list[0] + dx*(i+loop*10) for i in range(10)]
                    y_list2 = [param.y_list[0] + dy*(i+loop*10) for i in range(10)]
                    time_list2 = [param.time_list[0]+dt*(i+loop*10) for i in range(10)]
                    loop += 1
                    
                    for i in range(10):
                        if param.time_list[-1]< time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            self.stop_flag = True
                        else:
                            break

                if time_list2 != []:
                    time_list3 = [datetime.fromtimestamp(time_list2[i]) for i in range(len(time_list2))]
                    astro_time = Time(time_list3)


                    ret = self.calc.coordinate_calc(x_list2, y_list2, astro_time,
                                                    param.coord, param.off_az, param.off_el, 
                                                    param.hosei, param.lamda, self.press,
                                                    self.out_temp, self.out_humi, param.limit, param.rotation)
                    if param.rotation:
                        ret[0] = self.negative_change(ret[0])

                else:
                    limit_flag = True
                    
                """limit check"""
                for i in range(len(time_list2)):
                    if not -240*3600<ret[0][i]<240*3600 or not 0.<=ret[1][i]<90*3600.:
                        print("reaching soft limit : ", )
                        print("az : ", ret[0][i]/3600., "[deg]")
                        print("el : ", ret[1][i]/3600., "[deg]")
                        self.stop_flag = True
                        limit_flag = True
                        break
                    else:
                        pass
                    limit_flag = False

                if not limit_flag:
                    msg.x_list = ret[0]
                    msg.y_list = ret[1]
                    msg.coord = param.coord
                    msg.time_list = time_list2
                    msg.from_node =node_name
                    msg.timestamp = time.time()
                    self.pub.publish(msg)
                    print("msg", msg)
                    print("publish ok")
                else:
                    limit_flag = False
                    move_flag = False
            else:                
                loop = 0
                self.param = ""
                pass
            time.sleep(0.1)
        return

    def negative_change(self, az_list):
        print(az_list)
        
        
        if all((-240*3600<i< 240*3600. for i in az_list)):
            pass
        elif all((i<-110*3600. for i in az_list)):
            az_list = [i+360*3600. for i in az_list]
        elif all((i>110*3600. for i in az_list)):
            az_list = [i-360*3600. for i in az_list]
        elif all((-270*3600<i< 270*3600. for i in az_list)):
            pass
        elif any((i>=340*3600. for i in az_list)) and any((i<=20*3600. for i in az_list)):
            az_list = [i-360*3600. if i>=340*3600 else i for i in az_list]
        else:
            print("Az limit error.")
        return az_list
        
if __name__ == "__main__":
    rospy.init_node(node_name)
    azel = azel_list()
    azel.create_azel_list()

