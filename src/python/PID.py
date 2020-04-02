class PID:
    """
    Discrete PID control
    """

    def __init__(self, Kp: float = 1.0, Ti: float = 0, Td: float = 0, I_max: float = 1, I_min: float = -1, error_alpha: float = 0.1):

        # parameters
        self.Kp: float = Kp
        self.Ti: float = Ti
        self.Td: float = Td
        self.I_max: float = I_max
        self.I_min: float = I_min
        self.error_alpha = error_alpha

        # state
        self.P: float = 0
        self.I: float = 0
        self.D: float = 0
        self.derror_dt: float = 0
        self.set_point: float = 0
        self.lastUpdateTime: float = None
        self.lastError: float = None

    def reset(self):
        self.P = 0
        self.I = 0
        self.D = 0
        self.derror_dt = 0
        self.lastUpdateTime = None

    def status(self):
        return {
            "P": self.P,
            "I": self.I,
            "D": self.D,
            "dedt": self.derror_dt,
        }

    def update(self, t: float, current_value: float):
        """
        Calculate PID output value for given reference input and feedback
        """

        error = current_value - self.set_point

        if self.lastUpdateTime is None:
            self.lastError = error
            self.lastUpdateTime = t
            return None

        dt = t - self.lastUpdateTime

        self.P = self.Kp * error

        # TODO: use second order deriviate
        derror_dt = (error - self.lastError) / dt
        self.derror_dt += self.error_alpha * (derror_dt - self.derror_dt) / dt
        self.D = self.Td * self.derror_dt

        if self.Ti > 0:
            self.I += self.P * dt / self.Ti
            if self.I > self.I_max:
                self.I = self.I_max
            elif self.I < self.I_min:
                self.I = self.I_min

        self.lastUpdateTime = t
        self.lastError = error

        return self.P + self.I + self.D

    # FIXME: setSetPoint()
    def setPoint(self, set_point):
        """
        Initialize the setpoint of PID
        """
        self.set_point = set_point
        self.reset()

    def setKp(self, Kp):
        self.Kp = Kp

    def setTi(self, Ti):
        self.Ti = Ti

    def setTd(self, Td):
        self.Td = Td

    # FIXME: getSetPoint()
    def getPoint(self):
        return self.set_point
