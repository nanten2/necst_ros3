#!/usr/bin/env python3

name = "antenna_move"

import rospy
import std_msgs.msg
import necst.msg
import time
import numpy


class Antenna(object):

    parameters = {
        'az_list':[],
        'el_list':[],
        'start_time_list':[],
        }

    lock_az = False
    lock_el = False

    def __init__(self):
        self.start_time = time.time()

        rospy.Subscriber('list_azel', necst.msg.List_coord_msg, self.set_parameter, queue_size=1000)
        rospy.Subscriber('stop_cmd', std_msgs.msg.Bool, self.stop_move, queue_size=1)

        topic_from_az = rospy.Subscriber(
                name = "/antenna/az_lock",
                data_class = std_msgs.msg.Bool,
                callback = self.set_flag_az,
                queue_size = 1,
            )

        topic_from_el = rospy.Subscriber(
                name = "/antenna/el_lock",
                data_class = std_msgs.msg.Bool,
                callback = self.set_flag_el,
                queue_size = 1,
            )
        
        self.topic_az = rospy.Publisher(
                name = "/antenna/az_cmd",
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )
        
        self.topic_el = rospy.Publisher(
                name = "/antenna/el_cmd",
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )

        pass

    def set_flag_az(self, req):
        self.lock_az = req.data
        return

    def set_flag_el(self, req):
        self.lock_el = req.data
        return

    def set_parameter(self, req):
        if not req.time_list:
            return

        self.list_coord = req.coord
        if self.start_time < req.time_list[0]:
            if self.parameters['start_time_list'] != []:
                time_len = len(self.parameters['start_time_list'])
                for i in range(time_len):
                    __tmp = self.parameters
                    if req.time_list[0]< __tmp['start_time_list'][-1]:
                        del __tmp['az_list'][-1]
                        del __tmp['el_list'][-1]
                        del __tmp['start_time_list'][-1]
                        self.parameters = __tmp
                    else:
                        break
            else:
                pass

            tmp_ = self.parameters
            tmp_['az_list'].extend(req.x_list)
            tmp_['el_list'].extend(req.y_list)
            tmp_['start_time_list'].extend(req.time_list)
            self.parameters = tmp_
        else:
            pass
        return

    def time_check(self, st):
        ct = time.time()

        if ct - st[-1] >= 0:
            rospy.loginfo('!!!azel_list is end!!!')
            return
        return ct

    def comp(self):
        """
        DESCRIPTION
        ===========
        This function determine target Az and El from azel_list
        """
        param = self.parameters
        st = param['start_time_list']
        if st == []:
            return

        ct = self.time_check(st)
        if ct == None or param['az_list'] == []:
            return

        else:
            try:
                num = numpy.where(numpy.array(st) > ct)[0][0]
                if len(param['az_list']) > num:
                    if num == 0:
                        az_1 = az_2 =  param['az_list'][num]
                        el_1 = el_2 =  param['el_list'][num]
                        st2 = st[num]
                    else:
                        az_1 = param['az_list'][num-1]
                        az_2 = param['az_list'][num]
                        el_1 = param['el_list'][num-1]
                        el_2 = param['el_list'][num]
                        st2 = st[num-1]
                else:
                    return
                return (az_1,az_2,el_1,el_2,st2)
            except Exception as e:
                rospy.logerr(e)
                return

    def act_azel(self):
        while True:
            if self.lock_az or self.lock_el:
                time.sleep(0.1)
                continue

            ret = self.comp()
            if ret == None:
                time.sleep(0.1)
                continue
            else:
                hensa_az = ret[1] - ret[0]
                hensa_el = ret[3] - ret[2]
                current_time = time.time()
                start_time = ret[4]
                tar_az = ret[0] + hensa_az*(current_time-start_time)*10
                tar_el = ret[2] + hensa_el*(current_time-start_time)*10

                self.topic_az.publish(tar_az/3600.)
                self.topic_el.publish(tar_el/3600.)
        return

    def stop_move(self, req):
        if time.time() - self.start_time <1:
            return
        if req.data:
            self.parameters['az_list'] = []
            self.parameters['el_list'] = []
            self.parameters['start_time_list'] = []
        return


if __name__ == "__main__":
    rospy.init_node(name)
    ant = Antenna()
    ant.act_azel()

