###########################################################
#
#   FILENAME:       open_loop.py
#
#   DESCRIPTION:    Simple open-loop control model
#                     Assumes constant velocity from the
#                     drive system 
#
#
###########################################################

import math
import random
import matplotlib.pyplot    as plt
import numpy                as np
import src.models.dc_motor  as dc_motor
import src.models.wheel     as wheel

def sim():

    # Simulation constraints
    time_step_s             = 0.01  # Simulate the robot in time increments of 10 milliseconds
    motor_max_rpm           = 120   # Max RPM of the motor is 120 RPM
    steady_state_condition  = 5     # The robot needs to sit almost still for 5 seconds

    # Model handles
    motor_actuators     = dc_motor.DC_Motor(time_step_s, motor_max_rpm)

    # Make arrays to store the simulation in

    target_line         = [10]  # Target distance in meters
    estimated_position  = [0]   # Where the system thinks it is
    actual_position     = [0]   # Where the system actually is
    velocity            = [0]   # Current speed of the robot
    timestamp           = [0]   # Time of the simulation
    control_effort      = [0]   # Signal the controller passes to actuators
    solution_drift      = [0]   # How far off is estimated position from actual position

    target_reached      = 0     # Time at which the target was actually met

    # Using an open loop velocity model, we estimate that with the peak RPM
    #   the robot will always travel at a constant speed

    wheel_rps           = (motor_max_rpm / 60) / wheel.gear_ratio
    estimated_velocity  = wheel_rps * wheel.circumference_meters
    # Assume velocity can only be measured with 0.01 m/s accuracy
    estimated_velocity  = (int(estimated_velocity * 100) / 100.0) + random.gauss(0,0.01)

    travel_time         = target_line[-1] / estimated_velocity
    print(travel_time)

    # Run the simulation as long as the steady state critera hasn't been met
    steady_state        = False
    while(not steady_state):

        # Update the timestamp from the previous timestamp
        cur_time        = timestamp[-1] + time_step_s

        # If the current timestamp is below the travel time, move forward
        if(cur_time <= travel_time):
            # Move the robot forward
            distance_forward    = motor_actuators.rotate(dc_motor.DC_Motor.motor_directions["Forwards"])

            # Calculate the actual velocity 
            actual_velocity     = distance_forward / time_step_s

            # Estimate current location
            estimated_location  = estimated_position[-1] + estimated_velocity * time_step_s

            # Calculate the actual location
            actual_location     = actual_position[-1] + distance_forward

            # Calculate solution drift
            percent_drift       = ((estimated_location - actual_location) / actual_location) * 100.0

            # Append the simulation arrays
            target_line.append(target_line[-1])             # Target Line's value never changes, but needs to be as long as the sim
            estimated_position.append(estimated_location)
            actual_position.append(actual_location)
            velocity.append(actual_velocity)
            timestamp.append(cur_time)
            control_effort.append(1)
            solution_drift.append(percent_drift)

        # If we think we've ran out of travel time
        else:
            # Keep updating actual position based upon slowdown rate
            distance_forward    = motor_actuators.rotate(dc_motor.DC_Motor.motor_directions["Sustain"])
            actual_location     = actual_position[-1] + distance_forward

            # Append the simulation arrays, just updated everything to what they were
            target_line.append(target_line[-1])                 # Target Line's value never changes, but needs to be as long as the sim
            estimated_position.append(estimated_position[-1])
            actual_position.append(actual_location)
            velocity.append(velocity[-1])
            solution_drift.append(solution_drift[-1])

            # These two sim arrays are special
            timestamp.append(cur_time)
            control_effort.append(0)

        # Determine if we've reach steady state, if that much time as passed to start checking
        if(cur_time >= steady_state_condition):
            sample_size = steady_state_condition / time_step_s
            # Get the standard deviation
            standard_deviation  = np.std(estimated_position[int(0-sample_size):])

            # If the standard deviation is 0.01% of the target distance, we've reached steady state
            if (standard_deviation < (target_line[-1] * 0.0001)):
                steady_state = True

        # If the simulation has gone on longer than 360 seconds, enough is enough
        if( cur_time > 360):
            steady_state = True

    # Plot the results from the simulation

    # Plot the results from the simulation

    fig, ax1 = plt.subplots()

    # ax2 = ax1.twinx()

    ax1.plot(timestamp, target_line,            'r',    label="Target Position")
    ax1.plot(timestamp, estimated_position,     'm',    label="Estimated Position")
    ax1.plot(timestamp, actual_position,        'c',    label="Actual Position")
    # ax2.plot(timestamp, solution_drift,         'k',    label="Solution Drift")
    ax1.plot(timestamp, control_effort,         'b',    label="Control Effort")

    ax1.legend()
    # ax2.legend()

    ax1.set_ylabel("Position (m)")
    # ax2.set_ylabel("Solution % Deviation")

    ax1.set_xlabel("Time (s)")
    ax1.set_title("Open Loop Solution")

    plt.show()


