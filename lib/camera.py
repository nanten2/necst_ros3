#!/usr/bin/env python3

import subprocess
import time
import datetime

class controller(object):

    def _send_gphoto2_(self, arg):
        cmd = 'sudo gphoto2' + arg
        subprocess.call(cmd, shell=True)
        return

    def _return_gphoto2_(self, arg):
        cmd = 'sudo gphoto2' + arg
        ret = subprocess.check_output(cmd, shell=True)
        return ret

    def detect_camera(self):
        ret = self._return_gphoto2_(' --auto-detect')
        return ret

    def query_mode(self):
        ret = self._return_gphoto2_(' --get-config expprogram')
        if 'Current: P' in ret:
            mode = 'P'
        elif 'Current: M' in ret:
            mode = 'S'
        elif 'current: M' in ret:
            mode = 'M'
        else:
            mode = 'unknown'
        return mode

    def set_time(self, clock='now'):
        if clock=='now':
            self._send_gphoto2_(' --set-config datetime=now')
        else:
            tm = datetime.datetime.strptime(clock, '%Y%m%d%H%M%S')
            unix = int(time.mktime(tm.timetuple()))
            self.send_gphoto2_(' --set-config datetime='+str(unix))
        return

    def query_time(self):
        ret = self._return_gphoto2_(' --get-config datetime')
        ind = ret.find('Current:')
        unix = int(ret[ind+9:ind+19])
        systime = datetime.datetime.utcfromtimestamp(unix)
        utc = systime.strftime('%Y/%m/%d %H:%M:%S')
        return utc

    def query_battery(self):
        ret = self._return_gphoto2_(' --get-config batterylevel')
        ind = ret.find('Current:')
        battery = str(ret[ind+9:ind+13])
        return battery

    def set_whitebalance(self, white='SKY'):
        if white=='AUTO':
            self._send_gphoto2_(' --set-config whitebalance=0')
        elif white=='SKY':
            self._send_gphoto2_(' --set-config whitebalance=1')
        else:
            print('Cannot set = ' + white)
            print('Only use AUTO, SKY or LIGHT')
        return

    def query_whitebalance(self):
        ret = self._return_gphoto2_(' --get-config whitebalance')
        ind0 = ret.find('Current:')
        ind1 = ret.find('Choice: 0')
        ind2 = ret.find('Choice: 1')
        ind3 = ret.find('Choice: 4')
        wb = str(ret[ind0+9:ind0+15])
        if wb in ret[ind1+10:ind1+16]:
            white = 'AUTO'
        elif wb in ret[ind2+10:ind2+16]:
            white = 'SKY'
        else:
            white = 'unknown'
        return white

    def shutter(self):
        self._send_gphoto2_(' --capture-image')
        return

    def shutter_download(self, filename='capture/hope.jpg'):
        self._send_gphoto2_(' --capture-image-and-download --filename '+filename)
        return str(filename)

    def set_crop(self, crop='1.3x'):
        if crop == 'DX':
            self._send_gphoto2_(' --set-config d030=0')
        elif crop == '1.3x':
            self._send_gphoto2_(' --set-config d030=1')
        else:
            print('Cannot set = ' + crop)
            print('Only use DX or 1.3x')
        return

    def query_crop(self):
        ret = self._return_gphoto2_(' --get-config d030')
        ind0 = ret.find('current:')
        cr = int(ret[ind0+9])
        if cr == 0:
            crop = 'DX'
        elif cr == 1:
            crop = '1.3x'
        else:
            crop = 'unknown'
        return crop
