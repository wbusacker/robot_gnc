###########################################################
#
#   FILENAME:       continuous_velocity.py
#
#   DESCRIPTION:    Model for a continuous velocity servo
#
###########################################################

import random
import src.models.wheel         as wheel
import src.models.vehicle_mass  as vm

class Continuous_Velocity:

    def __init__(self, sim_time_step, max_rpm):
        self.cur_rpm                = 0
        self.sim_time_step          = sim_time_step
        self.max_rpm                = max_rpm

        # Figure out what the motor axle torque is. Assume similar torque drop
        #   like the normal motor
        if(self.max_rpm == 100):
            self.stall_torque_Nm    = 2.10
        elif(self.max_rpm == 160):
            self.stall_torque_Nm    = 1.31
        elif(self.max_rpm == 240):
            self.stall_torque_Nm    = 0.88

        # Using the stall torque and the mass, calculate the ramp limit
        # Get the force from the stall torque. Assume ideal torque transfer
        wheel_force     = (self.stall_torque_Nm * wheel.gear_ratio) / wheel.radius_meters

        # Take into account each wheel
        vehicle_force   = wheel_force * wheel.count

        # Estimate acceleration
        accel = vehicle_force / vm.mass_kg

        # Calculate the distance traveled in 1 second
        traversed_m = 0.5 * accel

        # Calculate number of rotations to get there
        wheel_rotations = (traversed_m / wheel.circumference_meters)
        motor_rotations = wheel_rotations * wheel.gear_ratio

        # motor_rotations is the ramp limit in revolutions / second, convert to rpm
        self.rpm_ramp_limit = motor_rotations * 60

    # Returns the distance the robot as traveled in this time step
    def rotate(self, commanded_rpm):

        # Ramp velocity
        delta_rpm   = commanded_rpm - self.cur_rpm


        