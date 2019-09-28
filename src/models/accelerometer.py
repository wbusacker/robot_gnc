###########################################################
#
#   FILENAME:       accelerometer.py
#
#   DESCRIPTION:    Model of the LIS344ALH device
#
#
###########################################################

import random

class Accelerometer:
    def __init__(self, sim_time_step):
        self.dt             = sim_time_step
        self.last_velocity  = 0
        self.last_position  = 0

    def get_accel(self, position):
        # Derive instantaneous acceleration
        velocity    = (position     - self.last_position) / self.dt
        accel       = (velocity     - self.last_velocity) / self.dt

        # Add noise
        accel       += random.gauss(0, 0.00003 * 9.8)
        accel       += random.gauss(0,0.1225)

        # Update last know values
        self.last_position  = position
        self.last_velocity  = velocity

        return accel