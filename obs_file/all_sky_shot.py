#! /usr/bin/env python3

import datetime
import argparse
import opt_point
import sys
arg = sys.argv

# Info
# ----

name = 'ALL_SKY_SHOT'
description = 'Get all sky shot data'

# Default parameters
# ------------------

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)

# Main
# ====

opt = opt_point.opt_point_controller()
arg.append("")
if arg[1] == "r_az":
    _sort = "r_az"
    print("[{}]  START OBSERVATION IN REVERSE AZ SORT MODE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
elif arg[1] == "line_az":
    _sort = "line_az"
    print("[{}]  START OBSERVATION IN LINE AZ SORT MODE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
elif arg[1] == "line_el":
    _sort = "line_el"
    print("[{}]  START OBSERVATION IN LINE EL SORT MODE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
elif arg[1] != '':
    _sort = 'el'
    print("[{}]  START OBSERVATION IN EL SORT MODE".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))
else:
    _sort = 'az'
    print("[{}]  START OBSERVATION IN AZ SORT MODE (DEFALUT)".format(datetime.datetime.strftime(datetime.datetime.now(), "%H:%M:%S")))

opt.start_observation(sort=_sort)
