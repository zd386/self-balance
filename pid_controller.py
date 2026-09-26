class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(-15.0, 15.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = 0.0   # Added

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_measurement = 0.0   # Added

    def compute(self, measurement, dt):
        if dt <= 0:
            return 0.0

        error = self.setpoint - measurement

        self._integral += error * dt
        max_integral = 50.0
        self._integral = max(-max_integral, min(max_integral, self._integral))

        # Derivative based on measurement (prevents derivative kick)
        derivative = -(measurement - self._prev_measurement) / dt
        self._prev_measurement = measurement
        self._prev_error = error

        output = (
            (self.kp * error)
            + (self.ki * self._integral)
            + (self.kd * derivative)
        )

        low, high = self.output_limits
        output = max(low, min(high, output))
        return output

