#!/usr/bin/env python3
# coding:utf-8


# Configurations
# ==============
# Info
# ----

name = 'radio_pointing_line_9'
description = 'Do radio pointing'


# Default parameters
# ------------------
obsfile = ''
tau = 0.0
integmin = 8000
integmax = 9000
plot_mode = 'plot'
save_path = '/home/amigos/data/result_png'
# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--obsfile', type=str,
               help='absolute path for obsfile', required=True)
p.add_argument('--tau', type=float,
               help='tau. default=%.1f'%(tau))
p.add_argument('--integmin', type=int,
               help='integrange_min')
p.add_argument('--integmax', type=int,
               help='integrange_max')
p.add_argument('--plot_mode', type=str,
               help='plot mode : plot/savefig')
p.add_argument('--savepath', type=str,
               help='save path')
args = p.parse_args()

if args.obsfile is not None: obsfile = args.obsfile
if args.tau is not None: tau = args.tau
if args.integmin is not None: integmin = args.integmin
if args.integmax is not None: integmax = args.integmax
if args.plot_mode is not None: plot_mode = args.plot_mode
if args.savepath is not None: savepath = args.savepath

# Main
# ====
import os
import shutil
import datetime
import sys
sys.path.append("/home/amigos/ros/src/necst_ros3/scripts")
sys.path.append("/home/amigos/ros/src/necst_ros3/lib")
import time
import signal
import numpy
import doppler_nanten
dp = doppler_nanten.doppler_nanten()

WARN = "\033[31m"
END = "\033[0m\n"

obs_items = open("/home/amigos/necst-obsfiles/" + obsfile, 'r').read().split('\n')
obs = {}
print("[{}]  OBSFILE OPEN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
for _item in obs_items:
    print(_item)
    if _item.startswith('script;'): break
    _item = _item.split('#')[0]
    _key, _value = _item.split('=', 1)
    _key = _key.strip()
    _value = _value.strip()
    try:
        obs[_key] = eval(_value)
    #except NameError:
    except:
        try:
            obs[_key] = eval(_value, obs)
        except:
            obs[_key] = obs[_value]
            pass
    continue

integ = obs['exposure']
ra = obs['lambda_on']#on点x座標
dec = obs['beta_on']#on点y座標
offx = obs['lambda_off']#off点x座標
offy = obs['beta_off']#off点y座標
xgrid = obs["xgrid"] #offset of pointing(x)
ygrid = obs["ygrid"] #offset of pointing(y)
point_n = obs["N"] #number of line
#point_n = int(point_n / 2) + 1 #number of 1line
if obs['otadel'].lower() == 'y':
    offset_dcos = 1
else:
    offset_dcos = 0
if obs['lo1st_sb_1'] == 'U':#後半に似たのがあるけど気にしない               
   sb1 = 1
else:
    sb1 = -1
if obs['lo1st_sb_2'] == 'U':#後半に似たのがあるけど気にしない               
    sb2 = 1
else:
    sb2 = -1  
if obs['cosydel'].lower() == 'j2000' or obs['cosydel'].lower() == 'b1950':
    cosydel = 'EQUATORIAL'
elif obs['cosydel'].lower() == 'galactic':
    cosydel = 'GALACTIC'
elif obs['cosydel'].lower() == 'horizontal':
    cosydel = 'HORIZONTAL'
else:
    print(WARN + "[{}]  COSYDEL ERROR".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), planet) + END)
    sys.exit()

import v3_controller
con = v3_controller.controller()
import v3_reader
red = v3_reader.reader(False)
con.dome.tracking(True)

def handler(num, flame):
    print("*** SYSTEM STOP!! ***")
    con.antenna.stop()
    con.dome.tracking(False)
    time.sleep(1)
    sys.exit()
    return

signal.signal(signal.SIGINT, handler)

# Initial configurations
# ----------------------

datahome = '/home/amigos/data/'
timestamp = time.strftime('%Y%m%d%H%M%S')
dirname = 'n%s_%s_%s_cross_%s_pointing'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],obs['object'])
savedir = os.path.join(datahome, name, dirname)

print("[{}]  MAKE DIRECTORY".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
os.makedirs(savedir)

# Data aquisition
# ---------------

d1_list = []
d2_list = []
tdim6_list = []
date_list = []
tsys_list = []
thot_list = []
tcold_list = []
vframe_list = []
vframe2_list = []
lst_list = []
az_list = []
el_list = []
lamdel_list = []
betdel_list = []
tau_list = []
hum_list = []
tamb_list = []
press_list = []
windspee_list = []
winddire_list = []
sobsmode_list = []
mjd_list = []
secofday_list = []
subref_list = []
_2NDLO_list1 = []
_2NDLO_list2 = []


print("[{}]  START OBSERVATION".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

ra = obs['lambda_on']
dec = obs['beta_on']
con.antenna.onepoint_move(ra, dec, obs['coordsys'])

savetime = time.time()
num = 0
n = int(obs['nTest']) * 2
latest_hottime = 0

while num < n:
    p_n = 0
    while p_n < point_n:
        off_x = 0
        off_y = 0
        
        print("[{0}]  P_N {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), p_n))
        
        if num % 2 == 0:
            off_x = xgrid * (p_n - (int(point_n/2)))
        else:
            off_y = ygrid * (p_n - (int(point_n/2)))
        
        print("[{0}]  OBJECT RA {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), ra))
        print("[{0}]  OBJECT DEC {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), dec))
        
        print("[{0}]  OBSERVATION : {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), num))
        con.antenna.stop()
        print("[{}]  ANTENNA TRACKING START".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        con.antenna.onepoint_move(ra, dec, obs['coordsys'], off_x=off_x+obs["offset_Az"], off_y=off_y+obs["offset_El"], offcoord = cosydel,dcos=1)
        print("[{}]  ANTENNA MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

        _now = time.time()
        if _now > latest_hottime+60*obs['load_interval']:
            con.hot.position("IN")
            print("[{}]  HOT IN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
            hot = red.hot.position()
            while hot != "IN":
                print("[{}]  HOT MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
                hot = red.hot.position()
                time.sleep(0.5)
                    
            print("[{}]  ANTENNA TRACKING CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
            while round(red.antenna.az(), 4) != round(red.antenna.az_cmd(), 4) or round(red.antenna.el(), 4) != round(red.antenna.el_cmd(), 4):
                time.sleep(0.1)
                continue

            #temp = float(red.weather.cabin_temp())
            temp = float(30)
            print("[{0}]  TEMPERATURE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), round(temp, 2)))
            
            dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], 0, 0, offset_dcos, obs['coordsys'], integ*2+integ, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)#obs['cosydel']非対応
            con.spectrometer.oneshot(exposure=integ)
            time.sleep(1)
            d = [red.achilles.oneshot_dfs1(), red.achilles.oneshot_dfs2()]
            print("[{}]  GET SPECTRUM".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

            d1 = d[0]
            d2 = d[1]
            d1_list.append(d1)
            d2_list.append(d2)
            lamdel_list.append(0)
            betdel_list.append(0)
            tdim6_list.append([16384,1,1])
            tmp_time = time.time()
            tmp2 = datetime.datetime.fromtimestamp(tmp_time)
            tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
            date_list.append(tmp3)            
            thot_list.append(temp)
            vframe_list.append(dp1[0])
            vframe2_list.append(dp1[0])
            #lst_list.append(status.LST)
            az_list.append(red.antenna.az())
            el_list.append(red.antenna.el())
            tau_list.append(tau)
            hum_list.append(red.weather.out_humi())
            tamb_list.append(red.weather.out_temp())
            press_list.append(red.weather.pressure())
            windspee_list.append(red.weather.wind_speed())
            winddire_list.append(red.weather.wind_direction())
            sobsmode_list.append('HOT')
            #mjd_list.append(status.MJD)
            #secofday_list.append(status.Secofday)
            #subref_list.append(red.m2.position())
            latest_hottime = time.time()
            P_hot = numpy.sum(d1)
            tsys_list.append(0)
            _2NDLO_list1.append(dp1[3]['sg21']*1000)
            _2NDLO_list2.append(dp1[3]['sg22']*1000)
            pass
        
        
        con.hot.position("OUT")
        print("[{}]  HOT OUT".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        hot = red.hot.position()
        while hot != "OUT":
            print("[{}]  HOT MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
            hot = red.hot.position()
            time.sleep(0.5)
            continue

        con.antenna.onepoint_move(offx, offy, obs['coordsys'],off_x=obs["offset_Az"], off_y=obs["offset_El"],dcos=1)
        print("[{}]  ANTENNA MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        
        
        if latest_hottime > _now:
            pass
        else:
            dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], 0, 0, offset_dcos, obs['coordsys'], integ+integ, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)#obs['cosydel']非対応
       
        print("[{}]  ANTENNA TRACKING CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        while round(red.antenna.az(), 4) != round(red.antenna.az_cmd(), 4) or round(red.antenna.el(), 4) != round(red.antenna.el_cmd(), 4):
            time.sleep(0.1)
            continue

        #temp = float(red.weather.cabin_temp())
        temp = float(30)
        print("[{0}]  TEMPERATURE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), round(temp, 2)))

        con.spectrometer.oneshot(exposure=integ)
        time.sleep(1)
        d = [red.achilles.oneshot_dfs1(), red.achilles.oneshot_dfs2()]
        print("[{}]  GET SPECTRUM".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        
        d1 = d[0]
        d2 = d[1]
        d1_list.append(d1)
        d2_list.append(d2)
        tdim6_list.append([16384,1,1])
        tmp_time = time.time()
        tmp2 = datetime.datetime.fromtimestamp(tmp_time)
        tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
        date_list.append(tmp3)        
        thot_list.append(temp)
        vframe_list.append(dp1[0])
        vframe2_list.append(dp1[0])
        #lst_list.append(status.LST)
        az_list.append(red.antenna.az())
        el_list.append(red.antenna.el())
        tau_list.append(tau)
        hum_list.append(red.weather.out_humi())
        tamb_list.append(red.weather.out_temp())
        press_list.append(red.weather.pressure())
        windspee_list.append(red.weather.wind_speed())
        winddire_list.append(red.weather.wind_direction())
        sobsmode_list.append('OFF')
        #mjd_list.append(status.MJD)
        #secofday_list.append(status.Secofday)
        #subref_list.append(red.m2.position())
        P_sky = numpy.sum(d1)
        tsys = temp/(P_hot/P_sky-1)
        tsys_list.append(tsys)
        lamdel_list.append(0)
        betdel_list.append(0)
        _2NDLO_list1.append(dp1[3]['sg21']*1000)
        _2NDLO_list2.append(dp1[3]['sg22']*1000)

        con.antenna.stop()
        
        con.antenna.onepoint_move(ra, dec, obs['coordsys'], off_x = off_x+obs["offset_Az"], off_y = off_y+obs["offset_El"], offcoord = cosydel,dcos=1)
        print("[{}]  ANTENNA MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        
        print("[{}]  ANTENNA TRACKING CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        while round(red.antenna.az(), 4) != round(red.antenna.az_cmd(), 4) or round(red.antenna.el(), 4) != round(red.antenna.el_cmd(), 4):
            time.sleep(0.1)
            continue

        #temp = float(red.weather.cabin_temp())
        temp = float(30)
        print("[{0}]  TEMPERATURE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), round(temp, 2)))

        con.spectrometer.oneshot(exposure=integ)
        time.sleep(1)
        d = [red.achilles.oneshot_dfs1(), red.achilles.oneshot_dfs2()]
        print("[{}]  GET SPECTRUM".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

        d1 = d[0]
        d2 = d[1]
        d1_list.append(d1)
        d2_list.append(d2)
        tdim6_list.append([16384,1,1])
        tmp_time = time.time()
        tmp2 = datetime.datetime.fromtimestamp(tmp_time)
        tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
        date_list.append(tmp3)
        thot_list.append(temp)
        vframe_list.append(dp1[0])
        vframe2_list.append(dp1[0])
        #lst_list.append(status.LST)
        az_list.append(red.antenna.az())
        el_list.append(red.antenna.el())
        tau_list.append(tau)
        hum_list.append(red.weather.out_humi())
        tamb_list.append(red.weather.out_temp())
        press_list.append(red.weather.pressure())
        windspee_list.append(red.weather.wind_speed())
        winddire_list.append(red.weather.wind_direction())
        sobsmode_list.append('ON')
        #mjd_list.append(status.MJD)
        #secofday_list.append(status.Secofday)
        #subref_list.append(red.m2.position())
        tsys_list.append(tsys)
        _2NDLO_list1.append(dp1[3]['sg21']*1000)
        _2NDLO_list2.append(dp1[3]['sg22']*1000)
        if num % 2 == 0:
            lamdel_list.append(xgrid * (p_n - (int(point_n/2))))
            betdel_list.append(0)
        else:
            lamdel_list.append(0)
            betdel_list.append(ygrid * (p_n - (int(point_n/2))))    
            
        con.antenna.stop()
        
        p_n += 1    
    num += 1
    continue


# HOT->OFF->ON->OFF->...->ON->HOT
con.hot.position("IN")
print("[{}]  HOT IN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
hot = red.hot.position()
while hot != "IN":
    print("[{}]  HOT MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    hot = red.hot.position()
    time.sleep(0.5)
    continue

#temp = float(red.weather.cabin_temp())
temp = float(30)
print("[{0}]  TEMPERATURE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), round(temp, 2)))

dp1 = dp.set_track(obs['lambda_on'], obs['beta_on'], obs['vlsr'], obs['coordsys'], 0, 0, offset_dcos, obs['coordsys'], integ*2+integ, obs['restfreq_1']/1000., obs['restfreq_2']/1000., sb1, sb2, 8038.000000000/1000., 9301.318999999/1000.)#obs['cosydel']非対応
con.spectrometer.oneshot(exposure=integ)
time.sleep(1)
d = [red.achilles.oneshot_dfs1(), red.achilles.oneshot_dfs2()]
print("[{}]  GET SPECTRUM".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

d1 = d[0]
d2 = d[1]
d1_list.append(d1)
d2_list.append(d2)
lamdel_list.append(0)
betdel_list.append(0)
tdim6_list.append([16384,1,1])
tmp_time = time.time()
tmp2 = datetime.datetime.fromtimestamp(tmp_time)
tmp3 = tmp2.strftime("%Y/%m/%d %H:%M:%S")
date_list.append(tmp3)
thot_list.append(temp)
vframe_list.append(dp1[0])
vframe2_list.append(dp1[0])
#lst_list.append(status.LST)
az_list.append(red.antenna.az())
el_list.append(red.antenna.el())
tau_list.append(tau)
hum_list.append(red.weather.out_humi())
tamb_list.append(red.weather.out_temp())
press_list.append(red.weather.pressure())
windspee_list.append(red.weather.wind_speed())
winddire_list.append(red.weather.wind_direction())
sobsmode_list.append('HOT')
#mjd_list.append(status.MJD)
#secofday_list.append(status.Secofday)
#subref_list.append(red.m2.position())
latest_hottime = time.time()
P_hot = numpy.sum(d1)
tsys_list.append(0)
_2NDLO_list1.append(dp1[3]['sg21']*1000)
_2NDLO_list2.append(dp1[3]['sg22']*1000)

# ==================================
# save data
# ==================================
if obs['lo1st_sb_1'] == 'U':
    ul = 1
else:
    ul = -1
imagfreq1 = obs['obsfreq_1'] - ul*obs['if1st_freq_1']*2  
lofreq1 = obs['obsfreq_1'] - ul*obs['if1st_freq_1']*1

if obs['lo1st_sb_1'] == 'U':
    ul1_1 = +1
else:
    ul1_1 = -1
if obs['lo2nd_sb_1'] == 'U':
    ul1_2 = +1
else:
    ul1_2 = -1
if obs['lo3rd_sb_1'] == 'U':
    ul1_3 = +1
else:
    ul1_3 = -1
ul1 = ul1_1 * ul1_2 * ul1_3
cdelt1_1 = (-1)*ul1*0.079370340319607024 #[(km/s)/ch]
#dv1 = (300000*cdelt1_1)/obs['restfreq_1']
crpix1_1 = 8191.5 - obs['vlsr']/cdelt1_1 - (500-obs['if3rd_freq_1'])/0.061038881767686015

if obs['lo1st_sb_2'] == 'U':
    ul = 1
else:
    ul = -1
imagfreq2 = obs['obsfreq_2'] - ul*obs['if1st_freq_2']*2
lofreq2 = obs['obsfreq_2'] - ul*obs['if1st_freq_2']*1

if obs['lo1st_sb_2'] == 'U':
    ul2_1 = +1
else:
    ul2_1 = -1
if obs['lo2nd_sb_2'] == 'U':
    ul2_2 = +1
else:
    ul2_2 = -1
if obs['lo3rd_sb_2'] == 'U':
    ul2_3 = +1
else:
    ul2_3 = -1
ul2 = ul2_1 * ul2_2 * ul2_3
cdelt1_2 = (-1)*ul2*0.0830267951512371 #[(km/s)/ch]                                 
#dv2 = (300000*cdelt1_2)/obs['restfreq_2']
crpix1_2 = 8191.5 - obs['vlsr']/cdelt1_2 - (500-obs['if3rd_freq_2'])/0.061038881767686015

#d1list
read1 = {
    "OBJECT" : obs['object'],
    "BANDWID" : 1000000000, #デバイスファイルに追加
    "DATE-OBS" : date_list, 
    "EXPOSURE" : obs['exposure'],
    "TSYS" : tsys_list,
    "DATA" : d1_list,
    "TDIM6" : tdim6_list, #デバイスファイルに追加
    "TUNIT6" : 'counts', #デバイスファイルに追加
    "CTYPE1" : 'km/s', #デバイスファイルに追加
    "CRVAL1" : 0, #デバイスファイルに追加
    "CRPIX1" : crpix1_1, #デバイスファイルに追加
    "CDELT1" : cdelt1_1, #デバイスファイルに追加
    "CTYPE2" : 'deg', #未使用
    "CRVAL2" : 0, #未使用
    "CTYPE3" : 'deg', #未使用
    "CRVAL3" : 0, #未使用
    "T_VLSR" : 0, #未使用
    "OBSERVER" : obs['observer'],
    "SCAN" : 1, #要確認
    "OBSMODE" : obs['obsmode'],
    "MOLECULE" : obs['molecule_1'],
    "TRANSITI" : obs['transiti_1'],
    "TEMPSCAL" : 'TA', #未使用
    "FRONTEND" : 'nagoyaRX', #デバイスファイルに追加
    "BACKEND" : 'nagoyaDFS', #デバイスファイルに追加
    "THOT" : thot_list,
    "TCOLD" : 0, #tcold_list
    "FREQRES" : 0.06103515625, #デバイスファイルに追加[MHz]
    "TIMESYS" : 'UTC', #要確認
    "VELDEF" : 'RADI-LSR',
    "VFRAME" : vframe_list,
    "VFRAME2" : vframe2_list,
    "OBSFREQ" : obs['restfreq_1'], #restfreq_1
    "IMAGFREQ" : imagfreq1, #要計算
    "LST" : lst_list,
    "AZIMUTH" : az_list,
    "ELEVATIO" : el_list,
    "TAU" : tau_list,
    "HUMIDITY" : hum_list,
    "TAMBIENT" : tamb_list,
    "PRESSURE" : press_list,
    "WINDSPEE" : windspee_list,
    "WINDDIRE" : winddire_list,
    "BEAMEFF" : 1, #未使用
    "RESTFREQ" : obs['restfreq_1'],
    "SIG" : 'T', #未使用
    "CAL" : 'F', #未使用
    "SOBSMODE" : sobsmode_list,
    "QUALITY" : 1, #未使用
    "AOSLEN" : 0.04, #未使用
    "LOFREQ" : lofreq1, #要計算
    "SYNTH" : 8038.000000000,#要調査[MHz;IF1]2ndLO
    "FREQSWAM" : 0,#要調査
    "COORDSYS" : obs['coordsys'],
    "COSYDEL" : obs['cosydel'],
    "LAMDEL" : lamdel_list,
    "BETDEL" : betdel_list,
    "OTADEL" : obs['otadel'],
    "OTFVLAM" : 0,
    "OTFVBET" : 0,
    "OTFSCANN" : 0,
    "OTFLEN" : 0,
    "SUBSCAN" : 0, # 要実装
    "MJD" : mjd_list,
    "SECOFDAY" : secofday_list,
    "SIDEBAND" : obs['lo1st_sb_1'],
    "_2NDSB" : obs['lo2nd_sb_1'],
    "_3RDSB" : obs['lo3rd_sb_1'],
    "_2NDLO" : _2NDLO_list1,#ドップラーシフト込み
    "_3RDLO" : obs['lo3rd_freq_1'],
    "SUBREF" : subref_list,
    "LOCKSTAT" : 'F'#未使用
    }

#d2_list
#d1list                                                                        

read2 = {
    "OBJECT" : obs['object'],
    "BANDWID" : 1000000000, #デバイスファイルに追加
    "EXPOSURE" : obs['exposure'],
    "DATE-OBS" : date_list, 
    "TSYS" : tsys_list,
    "DATA" : d2_list,
    "TDIM6" : tdim6_list, #デバイスファイルに追加
    "TUNIT6" : 'counts', #デバイスファイルに追加
    "CTYPE1" : 'km/s', #デバイスファイルに追加 
    "CRVAL1" : 0, #デバイスファイルに追加
    "CRPIX1" : crpix1_2, #デバイスファイルに追加
    "CDELT1" : cdelt1_2, #デバイスファイルに追加
    "CTYPE2" : 'deg', #未使用
    "CRVAL2" : 0, #未使用
    "CTYPE3" : 'deg', #未使用
    "CRVAL3" : 0, #未使用
    "T_VLSR" : 0, #未使用
    "OBSERVER" : obs['observer'],
    "SCAN" : 1, #要確認
    "OBSMODE" : obs['obsmode'],
    "MOLECULE" : obs['molecule_2'],
    "TRANSITI" : obs['transiti_2'],
    "TEMPSCAL" : 'TA', #未使用
    "FRONTEND" : 'nagoyaRX', #デバイスファイルに追加
    "BACKEND" : 'nagoyaDFS', #デバイスファイルに追加                           
    "THOT" : thot_list,
    "TCOLD" : 0, #tcold_list                                                 
    "FREQRES" : 0.06103515625, #デバイスファイルに追加[MHz]                
    "TIMESYS" : 'UTC', #要確認                                                 
    "VELDEF" : 'RADI-LSR',
    "VFRAME" : vframe_list,
    "VFRAME2" : vframe2_list,
    "OBSFREQ" : obs['restfreq_2'],                                
    "IMAGFREQ" : imagfreq2, #要計算                                            
    "LST" : lst_list,
    "AZIMUTH" : az_list,
    "ELEVATIO" : el_list,
    "TAU" : tau_list,
    "HUMIDITY" : hum_list,
    "TAMBIENT" : tamb_list,
    "PRESSURE" : press_list,
    "WINDSPEE" : windspee_list,
    "WINDDIRE" : winddire_list,
    "BEAMEFF" : 1, #未使用                                                     
    "RESTFREQ" : obs['restfreq_2'],
    "SIG" : 'T', #未使用                                                       
    "CAL" : 'F', #未使用                                                       
    "SOBSMODE" : sobsmode_list,
    "QUALITY" : 1, #未使用                                                     
    "AOSLEN" : 0.04, #未使用                                                   
    "LOFREQ" : lofreq2, #要計算                                                
    "SYNTH" : 9301.318999999,#要調査[MHz;IF2]2ndLO                             
    "FREQSWAM" : 0,#要調査                                                     
    "COORDSYS" : obs['coordsys'],
    "COSYDEL" : obs['cosydel'],
    "LAMDEL" : lamdel_list,
    "BETDEL" : betdel_list,
    "OTADEL" : obs['otadel'],
    "OTFVLAM" : 0,
    "OTFVBET" : 0,
    "OTFSCANN" : 0,
    "OTFLEN" : 0,
    "SUBSCAN" : 0, # 要実装                                                    
    "MJD" : mjd_list,
    "SECOFDAY" : secofday_list,
    "SIDEBAND" : obs['lo1st_sb_2'],
    "_2NDSB" : obs['lo2nd_sb_2'],
    "_3RDSB" : obs['lo3rd_sb_2'],
    "_2NDLO" : _2NDLO_list2,#ドップラーシフト込み                              
    "_3RDLO" : obs['lo3rd_freq_2'],
    "SUBREF" : subref_list,
    "LOCKSTAT" : 'F'#未使用                                                    
    }


f1 = os.path.join(savedir,'n%s_%s_%s_cross_%s_pointing.fits'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],obs['object']))
f2 = os.path.join(savedir,'n%s_%s_%s_cross_%s_pointing.fits'%(timestamp ,obs['molecule_2'] ,obs['transiti_2'].split('=')[1],obs['object']))

sys.path.append("/home/amigos/ros/src/necst_ros3/lib")
import n2fits_write
n2fits_write.write(read1,f1)
n2fits_write.write(read2,f2)
print("[{}]  FITS WRITE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

shutil.copy("/home/amigos/ros/src/necst_ros3/lib/hosei_230.txt", savedir+"/hosei_copy")

import correct_fits# correct velocity
f1 = correct_fits.calc(f1)
f2 = correct_fits.calc(f2)

import pointing_line
pointing_line.analysis(f1, integ_mi=integmin, integ_ma=integmax, plot_mode=plot_mode, savepath_filename=save_path+"/result_cross_point{}.png".format(timestamp)) # f2?
print("[{}]  POINTING ANALYSIS".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

con.antenna.stop()
print("[{}]  END OBSERVATION".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
