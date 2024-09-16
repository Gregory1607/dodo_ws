class EncoderWrap:
    def __init__(self, encoder_min, encoder_max):
        self.encoder_min = encoder_min
        self.encoder_max = encoder_max
        self.mult = 0
        self.prev_encoder = 0
        self.curr_encoder = 0
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min

    def get_enc_tick(self, curr_value_from_mcu_enc):

        encoder = curr_value_from_mcu_enc

        # Check for low wrap
        if encoder < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap:
            self.mult += 1

        # Check for high wrap
        if encoder > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap:
            self.mult -= 1

        # Calculate the current encoder value
        self.curr_encoder = encoder + self.mult * (self.encoder_max - self.encoder_min)

        # Update previous encoder value
        self.prev_encoder = encoder

        return self.curr_encoder

    def reset(self):
        """
        Reset the encoder wrap state.
        """
        self.mult = 0
        self.prev_encoder = 0
        self.curr_encoder = 0

    def get_total_ticks(self):
        """
        Get the total number of ticks, including wraps.
        """
        return self.curr_encoder

    def get_wrap_count(self):
        """
        Get the number of times the encoder has wrapped.
        """
        return self.mult

