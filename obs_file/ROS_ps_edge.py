#! /usr/bin/env python
# coding:utf-8

import os
import shutil
import time
import math
import numpy
import datetime
import sys
sys.path.append("/home/amigos/ros/src/necst_ros3/lib")
import doppler_nanten
dp = doppler_nanten.doppler_nanten()
sys.path.append("/home/amigos/ros/src/necst_ros3/scripts")
import v3_controller
import v3_reader
import signal

def handler(num, flame):
    print("*** SYSTEM STOP!! ***")
    con.antenna.stop()
    con.dome.tracking(False)
    sys.exit()
    return

signal.signal(signal.SIGINT, handler)

WARN = "\033[31m"
END = "\033[0m\n"

# Configurations
# ==============
# Info
# ----

name = 'PS_EDGE_POINTING'
description = 'Get P/S spectrum'


# Default parameters
# ------------------
obsfile = ''
tau = 0.0
planet = ''
integmin = 5000
integmax = 10000

# Argument handler
# ================

import argparse

p = argparse.ArgumentParser(description=description)
p.add_argument('--obsfile', type=str,
               help='absolute path for obsfile')
p.add_argument('--tau', type=float,
               help='tau. default=%.1f'%(tau))
p.add_argument('--planet', type=str,
               help='planet_name or planet_number')
p.add_argument('--integmin', type=int,
               help='integrange_min')
p.add_argument('--integmax', type=int,
               help='integrange_max')

args = p.parse_args()

if args.obsfile is not None: obsfile = args.obsfile
if args.tau is not None: tau = args.tau
if args.planet is not None: planet = args.planet
if args.integmin is not None: integmin = args.integmin
if args.integmax is not None: integmax = args.integmax
print("[{0}]  OBJECT {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), planet))


# Main
# ====

con = v3_controller.controller()
red = v3_reader.reader(node=False)
con.dome.tracking(True)

obsdir = '/home/amigos/necst-obsfiles/'
obs_items = open(obsdir+obsfile, 'r').read().split('\n')
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
    except NameError:
        obs[_key] = obs[_value]
        pass
    continue

integ_on = obs['exposure']
integ_off = obs['exposure_off']
edge = math.fabs(obs['edge'])
if obs['otadel'].lower() == 'y':
    offset_dcos = 1
else:
    offset_dcos = 0

if obs['coordsys'].lower() == 'j2000' or obs['coordsys'].lower() == 'b1950':
    coord_sys = 'EQUATRIAL'
elif obs['coordsys'].lower() == 'galactic':
    coord_sys = 'GALACTIC'
elif obs['coordsys'].lower() == 'planet':
    coord_sys = 'PLANET'
    try:
        planet = int(planet)
        print("[{0}]  PLANET {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), planet))
    except:
        print("[{0}]  PLANET {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), planet))
        pass
    if isinstance(planet, str):
        planet_name = planet.lower()
        planet_number = {'mercury':1, 'venus':2, 'mars':4, 'jupiter':5, 'saturn':6, 'uranus': 7, 'neptune':8, 'pluto':9, 'moon':10, 'sun':11}
        planet = planet_number[planet_name]
    elif isinstance(planet, int):
        planet = int(planet)
    else:
        print(WARN + "[{}]  PLANET NAME ERROR".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), planet) + END)
        quit()
    if planet == 10:
        di = 1887#月の視直径[arcsec]
        r = di/2
    elif planet == 11:
        di = 1920#sun_r[arcsec]
        r =di/2
    else:
        print("[{}]  NEED SUN(10) OR MOON(11)".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        sys.exit()
else:
    print(WARN + "[{}]  COORDSYS ERROR".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), planet) + END)
    con.antenna.stop()
    con.dome.tracking(False)
    sys.exit()

if obs['cosydel'].lower() == 'j2000' or obs['cosydel'].lower() == 'b1950':
    cosydel = 'EQUATORIAL'
elif obs['cosydel'].lower() == 'galactic':
    cosydel = 'GALACTIC'
elif obs['cosydel'].lower() == 'horizontal':
    cosydel = 'HORIZONTAL'
else:
    print(WARN + "[{}]  COSYDEL ERROR".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), planet) + END)
    con.antenna.stop()
    con.dome.tracking(False)
    sys.exit()

if obs['lo1st_sb_1'] == 'U':
   sb1 = 1
else:
    sb1 = -1
if obs['lo1st_sb_2'] == 'U':
    sb2 = 1
else:
    sb2 = -1  


# Initial configurations
# ----------------------

datahome = '/home/amigos/data'
timestamp = time.strftime('%Y%m%d%H%M%S')
planet_number = {1:'mercury', 2:'venus', 4:'mars', 5:'jupiter', 6:'saturn', 7:'uranus', 8:'neptune', 9:'pluto', 10:'moon', 11:'sun'}
dirname = 'n%s_%s_%s_crossedge_%s_pointing'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],planet_number[planet])
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
subscan_list = []
lamdel_list = []
betdel_list = []

print("[{}]  START OBSERVATION".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

num = 0
n = int(obs['nSeq'])*4
latest_hottime = 0

while num < n: 
    gx = 0#進行方向の識別
    gy = 0
    if num%4 == 0:
        offset_x = -r - edge +obs["offset_Az"]
        offset_y = 0 +obs["offset_El"]
        line_point = obs['N']
        gx = 1
        subscan = 1
        place = 'left_edge'
    elif num%4 == 1:
        offset_x = +r + edge +obs["offset_Az"]
        offset_y = 0 +obs["offset_El"]
        line_point = obs['N']#int((edge*2)/obs['xgrid'])+1
        gx = 1
        subscan= 1
        place = 'right_edge'
    elif num%4 == 2:
        offset_x = 0 +obs["offset_Az"]
        offset_y = -r - edge +obs["offset_El"]
        line_point = obs['N']#int((edge*2)/obs['ygrid'])+1
        gy = 1
        subscan= 2
        place = 'lower_edge'
    elif num%4 == 3:
        offset_x = 0 +obs["offset_Az"]
        offset_y = +r + edge +obs["offset_El"]
        line_point = obs['N']#int((edge*2)/obs['ygrid'])+1
        gy = 1
        subscan= 2
        place = 'upper_edge'
    print("[{0}]  OBSERVATION : {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), place.upper()))

    con.antenna.stop()
    
    print("[{}]  ANTENNA TRACKING START".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    if coord_sys == 'PLANET':
        print("[{0}]  PLANET {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), planet))
        con.antenna.planet_move(planet, off_x=offset_x, off_y=offset_y, dcos = offset_dcos, offcoord = cosydel)
        print("[{0}]  OFF_X {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), offset_x))
        print("[{0}]  OFF_Y {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), offset_y))
        print("[{}]  ANTENNA MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    else:
        pass
    
    _now = time.time()
    con.hot.position("IN")
    print("[{}]  HOT IN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    hot = red.hot.position()
    while hot != "IN":
        print("[{}]  HOT MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        hot = red.hot.position()
        time.sleep(0.5)
            
    temp = float(red.weather.cabin_temp())
    print("[{0}]  TEMPERATURE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), round(temp, 2)))

    dp1 = 0
    con.spectrometer.oneshot(exposure=integ_off)
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
    vframe_list.append(dp1)#dp1[0])
    vframe2_list.append(dp1)#dp1[0])
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
    _2NDLO_list1.append(dp1)#dp1[3]['sg21']*1000)
    _2NDLO_list2.append(dp1)#dp1[3]['sg22']*1000)
    lamdel_list.append(0)
    betdel_list.append(0)
    subscan_list.append(subscan)


    con.hot.position("OUT")
    print("[{}]  HOT OUT".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    hot = red.hot.position()
    while hot != "OUT":
        print("[{}]  HOT MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        hot = red.hot.position()
        time.sleep(0.5)
        continue

    temp = float(red.weather.cabin_temp())
    print("[{0}]  TEMPERATURE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), round(temp, 2)))

    con.spectrometer.oneshot(exposure=integ_off)
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
    vframe_list.append(dp1)#dp1[0])
    vframe2_list.append(dp1)#dp1[0])
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
    _2NDLO_list1.append(dp1)#dp1[3]['sg21']*1000)
    _2NDLO_list2.append(dp1)#dp1[3]['sg22']*1000)
    lamdel_list.append(0)
    betdel_list.append(0)
    subscan_list.append(subscan)

    lp = 0
    while lp < line_point:
        print("[{0}]  OBSERVATION : {1} - {2}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), place.upper(), int(lp)+1))
        print("[{}]  OBSERVATION : ON".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
        con.antenna.stop()

        if coord_sys == 'EQUATRIAL':
            pass
        elif coord_sys == 'GALACTIC':
            pass
        elif coord_sys == 'PLANET':
            if num%4 == 1 or num%4 == 3 :
                print("[{}]  OBSERVATION : RIGHT OR UPPER".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
                off_x = offset_x + (-2*edge+obs['xgrid']*lp)*gx
                off_y = offset_y + (-2*edge+obs['ygrid']*lp)*gy
            else:
                print("[{}]  OBSERVATION : LEFT OR LOWER".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
                off_x = offset_x + (obs['xgrid']*lp)*gx
                off_y = offset_y + (obs['ygrid']*lp)*gy
            con.antenna.planet_move(planet, off_x = off_x,off_y = off_y, 
                            offcoord = cosydel,dcos = offset_dcos)
            print("[{0}]  OFF_X {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), offset_x))
            print("[{0}]  OFF_Y {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), offset_y))
            print("[{}]  ANTENNA MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

        temp = float(red.weather.cabin_temp())
        print("[{0}]  TEMPERATURE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), round(temp, 2)))

        con.spectrometer.oneshot(exposure=integ_on)
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
        vframe_list.append(dp1)#dp1[0])
        vframe2_list.append(dp1)#dp1[0])
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
        _2NDLO_list1.append(dp1)#dp1[3]['sg21']*1000)    
        _2NDLO_list2.append(dp1)#dp1[3]['sg22']*1000)
        lamdel_list.append(off_x)
        betdel_list.append(off_y)
        subscan_list.append(subscan)

        con.antenna.stop()
        lp += 1

    num += 1
    continue

con.hot.position("IN")
print("[{}]  HOT IN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
hot = red.hot.position()
while hot != "IN":
    print("[{}]  HOT MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    hot = red.hot.position()
    time.sleep(0.5)
    continue

temp = float(red.weather.cabin_temp())
print("[{0}]  TEMPERATURE {1}".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S"), round(temp, 2)))
        
con.spectrometer.oneshot(exposure=integ_off)
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
vframe_list.append(dp1)#dp1[0])
vframe2_list.append(dp1)#dp1[0])
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
P_hot = numpy.sum(d1)
tsys_list.append(0)
_2NDLO_list1.append(dp1)#dp1[3]['sg21']*1000)
_2NDLO_list2.append(dp1)#dp1[3]['sg22']*1000)
lamdel_list.append(0)
betdel_list.append(0)
subscan_list.append(subscan)


con.hot.position("OUT")
print("[{}]  HOT OUT".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))


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
#print(ul1)?
cdelt1_1 = (-1)*ul1*0.079370340319607024 #[(km/s)/ch]
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
#print(ul2)?
cdelt1_2 = (-1)*ul2*0.0830267951512371 #[(km/s)/ch]                           
crpix1_2 = 8191.5 - obs['vlsr']/cdelt1_2 - (500-obs['if3rd_freq_2'])/0.061038881767686015

#planet_number = {1:'mercury', 2:'venus', 4:'mars', 5:'jupiter', 6:'saturn', 7:'uranus', 8:'neptune', 9:'pluto', 10:'moon', 11:'sun'}
#d1list
read1 = {
    "OBJECT" : planet_number[planet],
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
    "SUBSCAN" : subscan_list,
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
    "OBJECT" : planet_number[planet],
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
    "SUBSCAN" : subscan_list,                                                  
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


f1 = os.path.join(savedir,'n%s_%s_%s_crossedge_%s_pointing.fits'%(timestamp ,obs['molecule_1'] ,obs['transiti_1'].split('=')[1],planet_number[planet]))
f2 = os.path.join(savedir,'n%s_%s_%s_crossedge_%s_pointing.fits'%(timestamp ,obs['molecule_2'] ,obs['transiti_2'].split('=')[1],planet_number[planet]))

import n2fits_write
n2fits_write.write(read1,f1)
n2fits_write.write(read2,f2)
print("[{}]  FITS WRITE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

import pointing_moon_edge
pointing_moon_edge.analysis(f1, integ_mi=integmin, integ_ma=integmax) # f2?
print("[{}]  POINTING ANALYSIS".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

print("[{}]  END OBSERVATION".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
