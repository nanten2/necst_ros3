#! /usr/bin/env python

import time
import os
import sys
import datetime
import argparse
sys.path.append("/home/amigos/ros/src/necst_ros3/scripts")
import v3_controller
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

time.sleep(15)
con.antenna.stop()
print("[{}]  ANTENNA STOP".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
con.antenna.drive("off")
time.sleep(1)
print("[{}]  ANTENNA DRIVE OFF".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

print("[{}]  FINALIZE END".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
