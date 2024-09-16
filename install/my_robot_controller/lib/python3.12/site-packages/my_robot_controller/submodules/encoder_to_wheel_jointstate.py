class WheelState:
    def __init__(self, radian_per_rotate, enc_tick_rotate, float_round):
        self.wheel_joint_state_value = 0.0
        self.full_rotation_angle = radian_per_rotate  # Angle in Radian for full rotation, 2*pi
        self.float_round = float_round  # Digits after float point
        self.encoder_ticks_per_rotation = enc_tick_rotate
        self.max_angle = 6.2832  # Approximately 2*pi

    def get_state(self, current_encoder, previous_encoder):
        # Calculate the change in encoder ticks
        delta_ticks = current_encoder - previous_encoder
        
        # Convert the change in ticks to radians
        delta_radians = (delta_ticks / self.encoder_ticks_per_rotation) * self.full_rotation_angle
        
        # Update the wheel joint state value
        self.wheel_joint_state_value += delta_radians
        
        # Normalize the angle to keep it within [-2π, 2π]
        if abs(self.wheel_joint_state_value) > self.max_angle:
            self.wheel_joint_state_value %= self.max_angle
            if self.wheel_joint_state_value > self.max_angle / 2:
                self.wheel_joint_state_value -= self.max_angle
            elif self.wheel_joint_state_value < -self.max_angle / 2:
                self.wheel_joint_state_value += self.max_angle
        
        # Round the result to the specified number of decimal places
        return round(self.wheel_joint_state_value, self.float_round)

    def reset(self):
        # Reset the wheel joint state value to 0
        self.wheel_joint_state_value = 0.0
