###########################################################
#
#   FILENAME:       dc_motor.py
#
#   DESCRIPTION:    Simple open-loop control model
#                     Assumes constant velocity from the
#                     drive system
#
#
###########################################################

import random
import src.models.wheel as wheel
import src.models.vehicle_mass  as vm

class DC_Motor:

    motor_directions = {
        "Forwards"   :   1,
        "Backwards"  :   2,
        "Sustain"    :   3,
    }

    # Object Constructor: Requires simulation time step in seconds and maximum rpm for the motor
    def __init__(self, sim_time_step, max_rpm):
        self.cur_rpm                = 0
        self.sim_time_step          = sim_time_step
        self.max_rpm                = max_rpm

        # Figure out what the motor axle torque is
        if(self.max_rpm == 120):
            self.stall_torque_Nm    = 1.67
        elif(self.max_rpm == 160):
            self.stall_torque_Nm    = 1.04
        elif(self.max_rpm == 240):
            self.stall_torque_Nm    = 0.7

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

    # Returns the distance the robot has traveled in this time step
    def rotate(self, direction):

        # Figure out which direction we're going
        negative = False
        if(direction == DC_Motor.motor_directions["Backwards"]):
            negative = True

        elif(direction == DC_Motor.motor_directions["Forwards"]):
            negative = False

        elif(direction == DC_Motor.motor_directions["Sustain"]):
            # If we're trying to stay in position figure out which direction we were
            #   going in and then go the opposite unless this next time step would 
            #   result in the motor stopping

            if  ( self.cur_rpm > self.rpm_ramp_limit):
                # We're faster than the ramp limit so we can safely go down
                negative = True
            elif( self.cur_rpm < (-1 * self.rpm_ramp_limit)):
                negative = False
            else:
                # We're in the middle region so return no rotation
                self.cur_rpm = 0
                return 0

        else:
            raise ValueError("Invalid Motor Direction")


        if(not negative):
            # print("Go Forward")
            # Check if we've exceeded max RPM
            if(self.cur_rpm != self.max_rpm):
                # print("Ramp Up")
                # Increase RPM
                self.cur_rpm += self.rpm_ramp_limit

                if self.cur_rpm > self.max_rpm:
                    self.cur_rpm = self.max_rpm

        else:
            # print("Go Backward")
            # Check if we've exceed min RPM
            if(self.cur_rpm != (-1 * self.max_rpm)):
                # print("Ramp Down")
                # Decrease RPM
                self.cur_rpm -= self.rpm_ramp_limit

                if self.cur_rpm < ( -1 * self.max_rpm):
                    self.cur_rpm = ( -1 * self.max_rpm)

        # Simulate some extra noise in the actual rpm. Assume std dev of 1 rpm in guassian distribution
        step_rpm    = self.cur_rpm + random.gauss(0,1)

        # Calculate wheel rotation amount
        wheel_rpm   = self.cur_rpm / wheel.gear_ratio
        wheel_rps   = wheel_rpm / 60
        rotatations = wheel_rps * self.sim_time_step

        # Calculate distance traveled
        traversed   = rotatations * wheel.circumference_meters
        return traversed
