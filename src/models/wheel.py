###########################################################
#
#   FILENAME:       wheel.py
#
#   DESCRIPTION:    Common model for the wheels
#
###########################################################

import math


count                   = 4                                 # Number of wheels

gear_ratio              = 1                                 # Number of motor turns to wheel turns

radius_meters           = 0.03                              # Radius of wheel
circumference_meters    = radius_meters * 2 * math.pi       # Circumference of wheel
