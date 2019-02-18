#! /usr/bin/env python3

import time
import datetime
import os
import shutil
import sys
import argparse

from astropy.coordinates import get_body,EarthLocation, SkyCoord
from astropy.time import Time
import astropy.units as u
nanten2 = EarthLocation(lat=-22.9699511*u.deg, lon=-67.60308139*u.deg, height=4863.84*u.m)

sys.path.append("/home/amigos/ros/src/necst_ros3/scripts")
import v3_controller
import v3_reader
import signal

# Info
# ----

name = 'ONESHOT'
description = 'GET ONESHOT DATA'

# Default parameters
# ------------------

star = ''
filename = ''

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)
p.add_argument('--star', type=str,
               help='Name of 1st magnitude star.(No space)')
p.add_argument('--name', type=str,
               help='save file name')

args = p.parse_args()

if args.star is None:
    print('!!STAR NAME IS NONE!!')
    sys.exit()
else:
    star = args.star
if args.name is not None: filename = args.name


# Main
# ====

star_list = []
planet_list = {"MERCURY":1, "VENUS":2, "MARS":4, "JUPITER":5, "SATURN":6, "URANUS":7, "NEPTUNE":8, "MOON":10, "SUN":11}
planet = 0

target = []


if not filename:
    filename = time.strftime("%Y%m%d%H%M%S")
dirname = "/home/amigos/data/opt/oneshot/"

if not os.path.exists(dirname):
    print("[{}]  MAKE DIRECTORY".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    os.makedirs(dirname)

#read star list
f = open("/home/amigos/ros/src/necst_ros3/lib/1st_star_list.txt")

line = f.readline()
while line:
    line = line.split()
    star_list.append(line)
    line = f.readline()

for i in range(len(star_list)):
    if star_list[i][0].upper() == star.upper():
        target.append(float(star_list[i][1]))
        target.append(float(star_list[i][2]))

if len(target) == 0:
    if star.upper() in planet_list:
        planet = star
        pass
    else:
        print('!!CAN NOT FIND THE NAME OF STAR!!')
        sys.exit()

con = v3_controller.controller()
red = v3_reader.reader(node=False)

def handler(num, flame):
    print("*** SYSTEM STOP!! ***")
    con.antenna.stop()
    con.dome.tracking(False)
    time.sleep(1)
    sys.exit()
    return


signal.signal(signal.SIGINT, handler)

print("[{}]  START OBSERVATION".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
con.dome.tracking(True)
con.antenna.stop()

print("[{}]  ANTENNA TRACKING START".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
if planet:
    now = datetime.datetime.utcnow()
    cplanet = get_body(planet, Time(now))
    cplanet.location = nanten2
    altaz = cplanet.altaz
    azelcoord = SkyCoord(altaz.az.deg, altaz.alt.deg, frame="altaz", unit="deg",obstime=Time(now), location=nanten2)
    radec = azelcoord.fk5

    con.antenna.onepoint_move(radec.ra.deg, radec.dec.deg, 'fk5', "", 0, 0, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)
    time.sleep(1)
    print("[{}]  ANTENNA MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    pass
else:
    now = datetime.datetime.utcnow()
    coo = SkyCoord(target[0],target[1], frame="fk5", unit="deg")
    coo.location = nanten2
    coo.obstime = Time(now)
    altaz = coo.altaz

    con.antenna.onepoint_move(target[0], target[1], 'fk5', "", 0, 0, offcoord="altaz", hosei='hosei_opt.txt', lamda = 0.5)
    time.sleep(1)
    print("[{}]  ANTENNA MOVING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    pass

print("[{}]  ANTENNA TRACKING CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
while round(red.antenna.az(), 4) != round(red.antenna.az_cmd(), 4) or round(red.antenna.el(), 4) != round(red.antenna.el_cmd(), 4):
    time.sleep(0.1)
    continue

print("[{}]  GET ONESHOT".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
con.camera.oneshot(filename)

con.dome.tracking(False)
con.antenna.stop()


while os.path.exists("/home/amigos/data/opt/"+filename+".jpg") == False:
    time.sleep(0.1)
    continue

shutil.move("/home/amigos/data/opt/"+filename+".jpg", "/home/amigos/data/opt/oneshot")

print("[{}]  END OBSERVATION".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
