class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D

        self.Integrator = None
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        
        self.set_point = None
        self.error = None
        self.lastUpdateTime = None


    def update(self, t, current_value):
        """
        Calculate PID output value for given reference input and feedback

        Ki is 1/tau
        """

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
            self.D = self.Kd * (error - self.error)

        #print error, self.error, self.Kd, self.D

        self.Integrator += self.Ki * error * dt

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I = self.Integrator * self.Ki

        self.error = error
        self.lastUpdateTime = t

        PID = self.P + self.I + self.D
        return PID

    def setPoint(self, t, set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator = 0
        self.lastError = None
        self.lastUpdateTime = t

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

