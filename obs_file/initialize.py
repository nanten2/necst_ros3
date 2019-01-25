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
red = v3_reader.reader(node=False)

con.antenna.drive("on")
print("[{}]  ANTENNA DRIVE ON".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
con.dome.door("OPEN")
print("[{}]  DOME DOOR OPEN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

if opt:
    con.dome.memb("OPEN")
    print("[{}]  DOME MEMB OPEN".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    time.sleep(10)

con.dome.tracking(True)
track = red.antenna.az() - red.dome.az()
while track > 1:
    print("[{}]  DOME TRACKING".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    time.sleep(5)
    track = red.antenna.az() - red.dome.az()
    continue

con.dome.tracking(False)
print("[{}]  DOME TRACKING END".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))


print("[{}]  INITIALIZE CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

print("[{}]  ANTENNA DRIVE CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
while red.antenna.drive() != "on":
    time.sleep(1)
    continue

print("[{}]  DOME DOOR CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
while red.dome.door() != "OPEN":
    time.sleep(1)
    continue

if opt:
    print("[{}]  DOME MEMB CHECK".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
    while red.dome.memb() != "OPEN":
        time.sleep(1)
        continue

print("[{}]  INITIALIZE END".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
