class PID:
    def __init__(self, Kp, Ki, Kd, highest_pwm, lowest_pwm):
        # PID constants
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # PWM limits
        self.highest_pwm = highest_pwm
        self.lowest_pwm = lowest_pwm

        # Internal state variables
        self.wheel_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.previous_error = 0.0
        self.applied_wheel_pwm = 0

    def get_pid_output(self, time_elapsed, target_velocity, current_velocity):
        """
        Calculate the PID output for wheel control.

        :param time_elapsed: Time since last update (seconds)
        :param target_velocity: Desired wheel velocity
        :param current_velocity: Actual wheel velocity
        :return: PWM value to be applied to the wheel
        """
        # Calculate error
        self.wheel_error = abs(target_velocity) - abs(current_velocity)

        # Proportional term
        p_term = self.Kp * self.wheel_error

        # Integral term
        self.integral += self.wheel_error * time_elapsed
        i_term = self.Ki * self.integral

        # Derivative term
        self.derivative = (self.wheel_error - self.previous_error) / time_elapsed
        d_term = self.Kd * self.derivative

        # Calculate total output
        self.applied_wheel_pwm = p_term + i_term + d_term

        # Apply PWM limits
        self.applied_wheel_pwm = max(self.lowest_pwm, min(self.highest_pwm, self.applied_wheel_pwm))

        # Reset integral term and PWM if target velocity is zero
        if target_velocity == 0.0:
            self.applied_wheel_pwm = 0
            self.integral = 0.0

        # Update previous error for next iteration
        self.previous_error = self.wheel_error

        return self.applied_wheel_pwm

    def reset(self):
        """
        Reset the PID controller state.
        """
        self.wheel_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.previous_error = 0.0
        self.applied_wheel_pwm = 0

    def update_constants(self, Kp, Ki, Kd):
        """
        Update PID constants.

        :param Kp: New proportional constant
        :param Ki: New integral constant
        :param Kd: New derivative constant
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def update_pwm_limits(self, highest_pwm, lowest_pwm):
        """
        Update PWM limits.

        :param highest_pwm: New maximum PWM value
        :param lowest_pwm: New minimum PWM value
        """
        self.highest_pwm = highest_pwm
        self.lowest_pwm = lowest_pwm
