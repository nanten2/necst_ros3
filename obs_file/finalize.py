#! /usr/bin/env python

import time
import sys
import datetime
import argparse
sys.path.append("/home/amigos/ros/src/necst_ros3/scripts")
import v3_controller
import v3_reader
import signal

def handler(signal, frame):
    print("*** SYSTEM STOP!! ***")
    con.antenna.stop()
    con.dome.tracking(False)
    con.dome.door("STOP")
    con.dome.memb("STOP")
    sys.exit()
    return

signal.signal(signal.SIGINT, handler)


# Info
# ----

name = 'FINALIZE'
description = 'FINALIZE OBSERVATION'

# Default parameters
# ------------------

snow = ''

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)
p.add_argument('--snow', type=str,
               help='For snow position. Need 1.')
args = p.parse_args()
if args.snow is not None:
    snow = args.snow
    target = "finalize (snow)"
else:
    target = "finalize (normal)"

# Main
# ====
print("[{}]  FINALIZE START".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

con = v3_controller.controller()
red = v3_reader.reader(node=False)

con.antenna.stop()
print("[{}]  ANTENNA STOP".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
con.dome.tracking(False)
print("[{}]  DOME TRACKING END".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
time.sleep(1)

if snow:
    con.antenna.onepoint_move(-90, 0.0001, limit=False)
    print("[{}]  ANTENNA MOVE (SNOW POSITION)".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
else:
    con.antenna.onepoint_move(0, 45)
    print("[{}]  ANTENNA MOVE (HOME POSITION)".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

con.dome.memb("CLOSE")
print("[{}]  DOME DOOR CLOSE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

con.dome.door("CLOSE")
print("[{}]  DOME DOOR CLOSE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

con.dome.move(90)
print("[{}]  DOME MOVE (HOME POSITION)".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
time.sleep(10)


print("[{}]  FINALIZE CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

print("[{}]  ANTENNA POSITION CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
if snow:
    while round(red.antenna.az(), 4) != -90.0 or round(red.antenna.el(), 3) != 0.0:
        time.sleep(1)
        continue
else:
    while round(red.antenna.az(), 5) != 0.0 or round(red.antenna.el(), 5) != 45.0:
        time.sleep(1)
        continue

con.antenna.stop()
print("[{}]  ANTENNA STOP".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
con.antenna.drive("off")
print("[{}]  ANTENNA DRIVE OFF".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

print("[{}]  DOME DOOR CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
while red.dome.door() != "CLOSE":
    time.sleep(1)
    continue

print("[{}]  DOME MEMB CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
while red.dome.memb() != "CLOSE":
    time.sleep(1)
    continue

print("[{}]  DOME AZ CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
while abs(round(red.dome.az()) - 90) < 1:
    time.sleep(1)
    continue


print("[{}]  FINALIZE END".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
