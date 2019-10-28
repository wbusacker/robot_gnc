import sys
if not((sys.version_info > (3, ))):
    print("FATAL\tThis Script needs Python 3")
    sys.exit()

import src.motor.open_loop
import src.motor.inertial_navigation
import src.motor.inertial_navigation_filtered

# src.motor.open_loop.sim()
# src.motor.inertial_navigation.sim()
src.motor.inertial_navigation_filtered.sim()