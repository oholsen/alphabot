class PID:
    """
    Discrete PID control
    """

    def __init__(self, Kp=1.0, Ti=0, Td=0, I_max=1, I_min=-1):

        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td

        self.I = 0 # None
        self.I_max = I_max
        self.I_min = I_min
        
        self.set_point = None
        self.error = None
        self.lastUpdateTime = None


    def update(self, t, current_value):
        """
        Calculate PID output value for given reference input and feedback

        Ki is 1/tau
        """

        # negative of error really
        error = self.set_point - current_value

        if self.lastUpdateTime is None:
            self.lastError = self.error
            self.lastUpdateTime = t
            return None

        
        dt = t - self.lastUpdateTime
        
        self.P = self.Kp * error

        # TODO: smoothing of derivative, external calculation?
        if self.error is None:
            self.D = 0
        else:
            self.D = self.Td * (error - self.error) / dt

        #print error, self.error, self.Kd, self.D

        self.I += self.P * dt / self.Ti

        if self.I > self.I_max:
            self.I = self.I_max
        elif self.I < self.I_min:
            self.I = self.I_min

        self.error = self.P
        self.lastUpdateTime = t

        return self.P + self.I + self.D


    def setPoint(self, t, set_point):
        """
        Initialize the setpoint of PID
        """
        self.set_point = set_point
        self.I = 0
        self.lastError = None
        self.lastUpdateTime = t

    def setKp(self, Kp):
        self.Kp = Kp

    def setTi(self, Ti):
        self.Ti = Ti

    def setTd(self, Td):
        self.Td = Td

    def getPoint(self):
        return self.set_point

