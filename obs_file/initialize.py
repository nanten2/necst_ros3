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

name = 'INITIALIZE'
description = 'INITIALIZE ANTENNA'

# Default parameters
# ------------------

opt = ''

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)
p.add_argument('--opt', type=str,
               help='For optical. Need 1.')

args = p.parse_args()

if args.opt is not None:
    opt = args.opt
    target = "initialize (opt)"
else:
    target = "initialize (nomal)"
# Main
# ====
print("[{}]  INITIALIZE START".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

con = v3_controller.controller()
con.antenna.drive("on")
print("[{}]  ANTENNA DRIVE ON".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
con.dome.door("OPEN")
print("[{}]  DOME DOOR OPEN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

if opt:
    con.dome.memb("OPEN")
    print("[{}]  DOME DOOR OPEN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    time.sleep(10)

con.dome.tracking(True)
print("[{}]  DOME TRACKING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
time.sleep(5)
con.dome.tracking(False)
print("[{}]  DOME TRACKING END".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

time.sleep(5)
print("[{}]  INITIALIZE END".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
