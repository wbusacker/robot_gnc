###########################################################
#
#   FILENAME:       pid.py
#
#   DESCRIPTION:    PID Abstraction Class
#
#
###########################################################

class PID:
    # Object Constructor: Requires simulation time step in seconds and maximum rpm for the motor
    def __init__(self, sim_time_step, kp, ki, kd):
        self.sim_time_step          = sim_time_step
        self.kp                     = kp
        self.ki                     = ki
        self.kd                     = kd
        self.error_integrator       = 0
        self.last_error             = None

    def process(self, error):
        output_effort   = 0

        # Proportional control
        output_effort   += error * self.kp

        # Integral Control  - Only turn on when error is small
        if(error < 1):
            self.error_integrator   += error
            output_effort           += self.error_integrator * self.sim_time_step * self.ki

        # Derivative Control
        if(self.last_error == None):
            # Just update the last error
            self.last_error = error
        else:
            derivative      = (error - self.last_error) / self.sim_time_step
            self.last_error = error
            output_effort   += derivative * self.kd

        return output_effort



