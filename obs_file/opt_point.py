#!/usr/bin/env python

import os
import time
import datetime
import sys
sys.path.append("/home/amigos/ros/src/necst_ros3/lib")
import opt_analy
import ccd_old as ccd
sys.path.append("/home/amigos/ros/src/necst_ros3/scripts")
import v3_controller
import v3_reader
import signal

from astropy.coordinates import SkyCoord,EarthLocation
from astropy.time import Time
import astropy.units as u
import calc_coord

nanten2 = EarthLocation(lat=-22.9699511*u.deg, lon=-67.60308139*u.deg, height=4863.84*u.m)


""" 
Notes about opt_point.py
------------------------
opt_point.py is used for optical pointing.
From the pointing list(pointing.list), select an observable object and move the antenna there with equatorial coordinate(J2000/FK5)
"""

class opt_point_controller(object):
    #reference : ccd.py, imageppm.cpp, imagepgm.cpp
    
    
    pointing_list = "/home/amigos/ros/src/necst_ros3/lib/pointing.list"
    tai_utc = 36.0 # tai_utc=TAI-UTC  2015 July from ftp://maia.usno.navy.mil/ser7/tai-utc.dat
    dut1 = 0.14708
    
    
    def __init__(self):
        self.con = v3_controller.controller()
        self.red = v3_reader.reader(node=False)
        self.calc = calc_coord.azel_calc()
        pass
    
    def handler(self, num, flame):
        print("!!ctrl+C!!")
        self.con.antenna.stop()
        self.con.dome.tracking(False)
        time.sleep(3.)
        sys.exit()
        return

    def create_table(self, sort ='az', hosei_opt="hosei_opt.txt"):
        """
        Returns
        -------
        target_list : list
            observable object list
        target_list:
        [0] : star number
        [1] : ra
        [2] : dec
        [3] : star magnitude
        [4] : azimuth
        """

        #create target_list
        f = open(self.pointing_list)
        line = f.readline()
        target_list = []
        
        #calculate mjd(now) and mjd(2000)
        now = datetime.datetime.now()
        _date = Time(now).mjd
        
        while line:
            _list = []
            line = line.replace(";", " ")
            line = line.split()
            
            #number(FK6)
            _list.append(line[0])
            
            #ra,dec(degree)
            now = datetime.datetime.now()
            _date = Time(now).mjd
            coord = SkyCoord(str(line[1])+'h'+ str(line[2]) +'m'+ str(line[3]) +'s',str(line[5]) + str(line[6]) + 'd' + str(line[7]) + 'm' + str(line[8]) + 's', frame='icrs')
            ra = coord.ra.deg + float(line[4])*(360./24.)/3600.*(_date - 51544)/36525.
            dec = coord.dec.deg + float(line[9])/3600.*(_date - 51544)/36525.
            
            _list.append(ra)
            _list.append(dec)
            _list.append(line[21]) #magnitude

            #store parameters in lists to use self.calc.coordinate_calc
            ra = [ra*3600.]
            dec = [dec*3600.]
            now = [now]

            ret = self.calc.coordinate_calc(ra, dec, now, 'fk5', 0, 0, hosei_opt, 2600, 5, 20, 0.07)
            _list.append(ret[0][0]) #az arcsec
            _list.append(ret[1][0])
            if _list[4] > 3600*180:#
                _list[4] = _list[4] -3600*360

            if sort == 'az' or sort == "r_az":
                if ret[1][0]/3600. >= 30 and ret[1][0]/3600. < 80:
                    num = len(target_list)
                    if num == 0:
                        target_list.append(_list) 
                    elif num == 1:
                        if target_list[0][4] < _list[4]:
                            target_list.append(_list)
                        else:
                            target_list.insert(0, _list)
                    else:
                        for i in range(num):
                            if target_list[i][4] > _list[4]:
                                target_list.insert(i, _list)
                                break
                            if i == num-1:
                                target_list.insert(num, _list)
                                pass

            elif sort == 'line_az':
                if ret[1][0]/3600. >= 35 and ret[1][0]/3600. <= 55:
                    num = len(target_list)
                    if num == 0:
                        target_list.append(_list) 
                    elif num == 1:
                        if target_list[0][4] < _list[4]:
                            target_list.append(_list)
                        else:
                            target_list.insert(0, _list)
                    else:
                        for i in range(num):
                            if target_list[i][4] > _list[4]:
                                target_list.insert(i, _list)
                                break
                            if i == num-1:
                                target_list.insert(num, _list)
                                pass                            
                            
            elif sort == 'line_el':
                if not (-10*3600. <= _list[4] <= +10*3600.):
                    pass
                elif ret[1][0]/3600. >= 30 and ret[1][0]/3600. < 80:
                    num = len(target_list)
                    if num == 0:
                        target_list.append(_list) 
                    elif num == 1:
                        if target_list[0][5] < _list[5]:
                            target_list.append(_list)
                        else:
                            target_list.insert(0, _list)
                    else:
                        for i in range(num):
                            if target_list[i][5] > _list[5]:
                                target_list.insert(i, _list)
                                break
                            if i == num-1:
                                target_list.insert(num, _list)
                                pass

            else:#el_sort
                if ret[1][0]/3600. >= 30 and ret[1][0]/3600. < 80:
                    num = len(target_list)
                    if num == 0:
                        target_list.append(_list) 
                    elif num == 1:
                        if target_list[0][5] < _list[5]:
                            target_list.append(_list)
                        else:
                            target_list.insert(0, _list)
                    else:
                        for i in range(num):
                            if target_list[i][5] > _list[5]:
                                target_list.insert(i, _list)
                                break
                            if i == num-1:
                                target_list.insert(num, _list)
                                pass                            
            
            line = f.readline()
            
        if sort == "r_az":
            re_az = [i for i in reversed(target_list)]
            target_list = re_az

        f.close()

        return target_list
    
    def start_observation(self, sort = 'az'):
        hosei_opt = "hosei_opt.txt"

        signal.signal(signal.SIGINT, self.handler)
        table = self.create_table(sort=sort, hosei_opt=hosei_opt)
        print("[{}]  CREATE OBJECT TABLE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        
        date = datetime.datetime.today()
        month = str("{0:02d}".format(date.month))
        day = str("{0:02d}".format(date.day))
        hour = str("{0:02d}".format(date.hour))
        minute = str("{0:02d}".format(date.minute))
        second = str("{0:02d}".format(date.second))
        data_name = "opt_"+str(date.year)+month+day+hour+minute+second

        print("[{}]  MAKE DIRECTORY".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        os.mkdir("/home/amigos/data/opt/" + data_name)

        print("[{0}]  HOSEI FILE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), hosei_opt))
        
        self.con.dome.tracking(True)
        
        for _tbl in table:
            now = datetime.datetime.utcnow()
            
             #store parameters in lists to use self.calc.coordinate_calc
            __ra = [_tbl[1]*3600.]
            __dec = [_tbl[2]*3600.]
            __now = [now]

            ret = self.calc.coordinate_calc(__ra, __dec, __now, 'fk5', 0, 0, hosei_opt, 0.5, 980, 260, 0.07)
            real_el = ret[1][0]/3600.
            print("[{0}]  OBJECT RA {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), ret[0][0]/3600.))
            print("[{0}]  OBJECT DEC {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), ret[1][0]/3600.))
            if real_el >= 30. and real_el < 79.5:
                print("[{}]  ANTENNA TRACKING START".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
                self.con.antenna.onepoint_move(_tbl[1], _tbl[2], "fk5",hosei=hosei_opt,lamda = 0.5, rotation = False)#lamda = 0.5 => 500
                print("[{}]  ANTENNA MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

                print("[{}]  ANTENNA TRACKING CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
                while round(self.red.antenna.az(), 4) != round(self.red.antenna.az_cmd(), 4) or round(self.red.antenna.el(), 4) != round(self.red.antenna.el_cmd(), 4):
                    time.sleep(0.1)
                    continue

                ret = self.calc.coordinate_calc(__ra, __dec, __now, 'fk5', 0, 0, hosei_opt, 2600, 5, 20, 0.07)
                try:
                    ccd.ccd_controller().all_sky_shot(_tbl[0], _tbl[3], ret[0][0]/3600., ret[1][0]/3600., data_name)
                    pass
                except Exception as e:
                    print("[{0}]  ERROR OCCURED : {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), e))
                    self.con.dome.tracking(False)
                    self.con.antenna.stop()
                    time.sleep(3)
                    sys.exit()

            else:
                print("[{0}]  THIS OBJECT IS OUT OF RANGE(EL)".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
                pass

        self.con.antenna.stop()
        time.sleep(3.)

        ###plot Qlook
        ###==========
        optdata_dir = '/home/amigos/data/opt/'
        try:
            print("[{}]  POINTING ANALYSIS".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
            opt_analy.opt_plot([optdata_dir+data_name], savefig=True, figname=data_name, interactive=True)
        except Exception as e:
            print("[{0}]  ERROR OCCURED : {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), e))

        try:
            import glob
            date = data_name[:8]
            file_list = glob.glob('{}{}*'.format(optdata_dir, date))
            opt_analy.opt_plot(file_list, savefig=True, interactive=True)     
            pass
        except Exception as e:
            print("[{0}]  ERROR OCCURED : {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), e))
        ###==========
        
        self.con.antenna.stop()
        print("[{}]  END OBSERVATION".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        return
    
    
