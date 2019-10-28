###########################################################
#
#   FILENAME:       intertial_navigation.py
#
#   DESCRIPTION:    Simple open-loop control model
#                     Assumes constant velocity from the
#                     drive system 
#
#
###########################################################

import  math
import  random
import  matplotlib.pyplot           as plt
import  numpy                       as np
import  src.models.dc_motor         as dc_motor
import  src.models.wheel            as wheel
import  src.models.accelerometer    as acl
from    src.models.ra_filter        import RA_Filter
from    src.models.iir_filter       import IIR_Filter
from    src.models.pid              import PID


def sim():

    # Simulation constraints
    time_step_s             = 0.01  # Simulate the robot in time increments of 10 milliseconds
    motor_max_rpm           = 120   # Max RPM of the motor is 120 RPM
    steady_state_condition  = 5     # The robot needs to sit almost still for 5 seconds
    filter_length           = int(0.5 / time_step_s)
    controller_threshold    = 0.01

    # Model handles
    motor_actuators     = dc_motor.DC_Motor(time_step_s, motor_max_rpm)
    controller          = PID(time_step_s, 0.5 , 1000, 2)
    accel               = acl.Accelerometer(time_step_s)
    filter              = RA_Filter(filter_length)
    # filter              = IIR_Filter()

    # Make arrays to store the simulation in

    target_line         = [10]  # Target distance in meters
    estimated_position  = [0]   # Where the system thinks it is
    estimated_velocity  = [0]
    estimated_acceleration = [0]
    actual_position     = [0]   # Where the system actually is
    timestamp           = [0]   # Time of the simulation
    control_effort      = [0]   # Signal the controller passes to actuators
    solution_drift      = [0]   # How far off is estimated position from actual position

    target_reached      = 0     # Time at which the target was actually met

    # Run the simulation as long as the steady state critera hasn't been met
    steady_state        = False
    while(not steady_state):

        # Update the timestamp from the previous timestamp
        cur_time            = timestamp[-1] + time_step_s

        # Calculate Error
        error               = target_line[-1] - estimated_position[-1]

        effort              = controller.process(error)

        # Create a deadband on the controller
        if( abs(effort) < controller_threshold):
            effort = 0

        # The DC motor only really allows forwards and backwards, so based upon the sign
        #   of the control effort, move in that direction.
        distance_traveled   = 0
        if(effort > 0):
            # Positive
            distance_traveled   = motor_actuators.rotate(dc_motor.DC_Motor.motor_directions["Forwards"])
        elif(effort < 0):
            # Negative
            distance_traveled   = motor_actuators.rotate(dc_motor.DC_Motor.motor_directions["Backwards"])
        else:
            distance_traveled   = motor_actuators.rotate(dc_motor.DC_Motor.motor_directions["Sustain"])
        

        # Update the true position
        current_position    = actual_position[-1] + distance_traveled

        # Feed the true position into the accelerometer model
        est_accel           = accel.get_accel(current_position)
        
        #Filter the detected acceleration
        est_accel           = filter.filter(est_accel)
        est_velocity        = estimated_velocity[-1] + (est_accel * time_step_s)
        est_position        = estimated_position[-1] + (est_velocity * time_step_s)

        # Update simulation arrays

        percent_drift       = abs(((current_position - est_position) / est_position) * 100)

        # Append the simulation arrays
        target_line.append(target_line[-1])             # Target Line's value never changes, but needs to be as long as the sim
        estimated_position.append(est_position)
        estimated_velocity.append(est_velocity)
        estimated_acceleration.append(est_accel)
        actual_position.append(current_position)
        timestamp.append(cur_time)
        control_effort.append(effort)
        solution_drift.append(percent_drift)

        # Determine if we've reach steady state, if that much time as passed to start checking
        if(cur_time >= steady_state_condition):
            sample_size = steady_state_condition / time_step_s
            # Get the standard deviation
            standard_deviation  = np.std(estimated_position[int(0-sample_size):])

            # If the standard deviation is 0.1% of the target distance, we've reached steady state
            if (standard_deviation < (target_line[-1] * 0.001)):
                steady_state = True

        # If the simulation has gone on longer than 360 seconds, enough is enough
        if( cur_time > 360):
            steady_state = True

    # Plot the results from the simulation

    fig, ax1 = plt.subplots()

    # ax2 = ax1.twinx()

    ax1.plot(timestamp, target_line,            'r',    label="Target Position")
    ax1.plot(timestamp, estimated_position,     'm',    label="Estimated Position")
    ax1.plot(timestamp, estimated_acceleration, 'y',    label="Estimated Accel")
    ax1.plot(timestamp, actual_position,        'c',    label="Actual Position")
    # ax2.plot(timestamp, solution_drift,         'k',    label="Solution Drift")
    ax1.plot(timestamp, control_effort,         'b',    label="Control Effort")

    ax1.legend()
    # ax2.legend()

    ax1.set_ylabel("Position (m)\nAcceleration (m/s/s)")
    # ax2.set_ylabel("Solution % Deviation")

    ax1.set_xlabel("Time (s)")
    ax1.set_title("Inertial Navigation / Dead Reckoning\n(Filtered)")

    # plt.legend([
    #     "Reference",
    #     "Estimated Position",
    #     "Estimate Accel",
    #     "Actual Position",
    #     "Solution Drift",
    #     "Control Effort"
    # ])


    plt.show()


