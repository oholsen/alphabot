import time
import asyncio
from PID import PID
import pigpio

CNTL = 7 # CE1
CNTR = 8 # CE0

MLIN1 = 12
MLIN2 = 13
MLEN = 6

MRIN1 = 20
MRIN2 = 21
MREN = 26

Servo1Pin = 27 # pitch neg up
Servo2Pin = 22 # yaw neg left


# Infrared detectors
DR = 16
DL = 19


pi = pigpio.pi()



class Proximity(object):
        
    def __init__(self, pin):
        self.pin = pin
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        #self.cb = pi.callback(pin, pigpio.RISING_EDGE) #, cb)

    def _cb(self, gpio, level, tick):
        sys.out.write('!')
            
    def isClose(self):
        return pi.read(self.pin) == 0


proximityLeft = Proximity(DL)
proximityRight = Proximity(DR)

class Servo(object):
    def __init__(self, pin, angle = 0, angleRange = 90, pulseWidthRange = 500):
        self.angleRange = angleRange
        self.pulseWidthRange = pulseWidthRange
        self.pin = pin
        pw = self._pulseWidth(angle)
        pi.set_servo_pulsewidth(self.pin, pw)

    def _pulseWidth(self, angle):
        # neutral is 1.5ms pulse
        # +-0.5ms is 90 degrees, depending on servo
        if angle > 0: angle = min(angle, self.angleRange)
        else: angle = max(angle, -self.angleRange)
        return 1500 + self.pulseWidthRange * angle / self.angleRange
            
    def setAngle(self, angle):
        pw = self._pulseWidth(angle)
        pi.set_servo_pulsewidth(self.pin, pw)

    def off(self):
        pi.set_servo_pulsewidth(self.pin, 0)
                
                
class Motor(object):

    def __init__(self, in1, in2, en):
        self.IN1 = in1
        self.IN2 = in2
        self.EN = en

        pi.set_mode(self.IN1, pigpio.OUTPUT)
        pi.set_mode(self.IN2, pigpio.OUTPUT)
        pi.set_mode(self.EN, pigpio.OUTPUT)

        self._stop()
        pi.set_PWM_frequency(self.EN, 500)

    def _stop(self):
        pi.write(self.IN1, 0)
        pi.write(self.IN2, 0)

    def _forward(self):
        pi.write(self.IN1, 1)
        pi.write(self.IN2, 0)

    def _backward(self):
        pi.write(self.IN1, 0)
        pi.write(self.IN2, 1)

    def stop(self):
        self._stop()
        pi.set_PWM_dutycycle(self.EN, 0)

    def setPower(self, power):
        if power >= 0:
            self._forward()
        else:
            self._backward()
        pi.set_PWM_dutycycle(self.EN, min(255, int(255.0 * abs(power) / 100)))

import sys
def cb(gpio, level, tick):
        sys.stdout.write('!')
        sys.stdout.flush()

class Counter(object):

        """Cannot detect slow speeds, depending on integration/count time"""
        
        def __init__(self, pin):
                self.cb = pi.callback(pin, pigpio.RISING_EDGE) #, cb)
                #self.lastTime = time.time()
                #self.lastCount = 0
                self.alpha = 0.9
                self._speed = 0
                self.counts = [(time.time(), 0)]
                self.n = 4

        def _cb(self, gpio, level, tick):
                sys.out.write('!')
                
        def speed(self):
                # alpha depends on update rate....
                # TODO: use time difference, automatic adjustment of alpha
                # guard against calling too often...
                t = time.time()
                count = self.cb.tally()

                if 0:
                        dcount = count - self.lastCount
                        dt = t - self.lastTime
                        self.lastTime = t
                        self.lastCount = count

                for th, ch in self.counts:
                        # was 0.5 and 25
                        if t - th > 0.3 or count - ch > 10:
                                tlast, clast = th, ch
                                break
                else:
                        tlast, clast = self.counts[-1]
                        
                self.counts.insert(0, (t, count))
                while len(self.counts) > self.n:
                        self.counts.pop()

                #print self.counts
                dcount = count - clast
                dt = t - tlast 
                if 0:
                        print()
                        print("speed", count, dcount, dt)
                        print()
                v = float(dcount) / dt
                self._speed = self.alpha * v + (1 - self.alpha) * self._speed
                #return self._speed, v, dcount, dt
                return self._speed

        def count(self):
                return self.cb.tally()
        
class MotorController:

        def __init__(self, name, motor, counter):
                self.name = name
                self.motor = motor
                self.counter = counter
                self.pid = PID(1, 0.3, 0, I_max=100, I_min=-100)
                self.lastUpdateTime = time.time()
                

                
        def update(self, debug=False):
                t = time.time()
                if t - self.lastUpdateTime < 0.100:
                        return
                
                countRate = self.counter.speed()
                #print self.counter.speed(), countRate, self.counter.times

                # TODO: can set direction in counter
                if self.pid.getPoint() < 0:
                        countRate = -countRate
                        
                power = self.pid.update(t, countRate)
                if power is None: return

                if debug:
                        print('%s  %6.2f %6.2f  %6.2f  %6.2f  %6.2f' % (self.name, countRate, self.pid.P, self.pid.I, self.pid.D, power), end=' ') 
                if power > 0:
                        power = min(power, 100)
                else:
                        power = max(power, -100)
                self.motor.setPower(power)
                if debug:
                        print(' %6.2f' % power)

        def setSpeed(self, speed):
             self.pid.setPoint(time.time(), speed)

        def getSpeed(self):
             return self.pid.getPoint()
				
        def stop(self):
            self.setSpeed(0)
            

left  = MotorController('L', Motor(MLIN1, MLIN2, MLEN), Counter(CNTL))
right = MotorController('R', Motor(MRIN2, MRIN1, MREN), Counter(CNTR))




class AlphaBot():

	def __init__(self):
		pass
		
	def isClose(self):
		return proximityLeft.isClose() or proximityRight.isClose()
	
	def setSpeed(self, speed):
		print("speed", speed)
		left.setSpeed(speed)
		right.setSpeed(speed)
		
	def getSpeed(self):
		return 0.5 * (left.getSpeed() + right.getSpeed())
		
	def setTurnSpeed(self, speed, turnSpeed):
		print("turn", speed, turnSpeed)
		left.setSpeed(speed + turnSpeed)
		right.setSpeed(speed - turnSpeed)
        
	def update(self):
		left.update()
		right.update()
		
		
alphabot = AlphaBot()


def testServo(pin, angleMin, angleMax, step=1):
    angle = 0
    dangle = +step
    servo = Servo(pin, angle)
    servo.setAngle(angle)
    while 1:
        time.sleep(0.2)
        #continue
        angle += dangle
        if angle > angleMax:
            dangle = -step
        elif angle < angleMin:
            dangle = step
        print('%d' % angle)
        servo.setAngle(angle)


def testMotorCounter():
    motor   = Motor(MLIN1, MLIN2, MLEN)
    counter = Counter(CNTL)
    power = 70
    motor.setPower(power)
    clast = counter.cb.tally()
    tlast = time.time()
    while 1:
        time.sleep(1.0)
        c = counter.cb.tally()
        t = time.time()
        dc = c - clast
        dt = t - tlast
        print(dc, dt, dc / dt)
        clast = c
        tlast = t

    while 1:
        time.sleep(0.1)
        #print power, '%5.2f  %5.2f, %d %.3f' % counter.speed()
        print(power, counter.speed())
                
def testMotor():
    left  = Motor(MLIN1, MLIN2, MLEN)
    right = Motor(MRIN2, MRIN1, MREN)
    stepSize = 10
    power = 0
    step = stepSize
        
    while 1:
        print(power)
        left.setPower(power)
        right.setPower(power)
        time.sleep(1.0)
        power += step
        if power > 100: step = -stepSize
        elif power < -100: step = stepSize

def testProximity():
    while 1:
        print(proximityLeft.isClose(), proximityRight.isClose())
        time.sleep(0.2)



def testSpeed(speed):
    
    left.setSpeed(0)
    right.setSpeed(0)

    tStart = time.time()
    # let counters init before applying power
    tSpeed = tStart + 2.0
    speedDuration = 5.0
    stopDuration = 0.3    
    dt = 0.050 # could adjust depending on speed... or integrate speed over longer time...
    stopped = True

    while True:
        t = time.time()
        
        if not stopped and t > tSpeed:
            left.stop()
            right.stop()
            print('Stop')
            
        if not stopped and t > tSpeed + stopDuration:
            stopped = True
            print('Stopped')
            
        if stopped and t > tSpeed:
            left.setSpeed(speed)
            right.setSpeed(speed)
            tSpeed = t + speedDuration
            speed = -speed
            stopped = False
            print('Start')
        
        
        # NEED TO STOP ENTIRELY BEFORE THROTTLING BACK UP
        
        
        time.sleep(dt)
        
        print('%.3f' % (t - tStart), end=' ') 
        left.update(True)
        right.update()

        if 0:
            # TODO: heading control, regulating difference to 0, adjusting speed for L/R
            cl = left.counter.count()
            cr = right.counter.count()
            #print '%.3f' % (t - tStart), cl, cr, cl - cr

@asyncio.coroutine
def controlLoop(dt):
	while True:
		yield from asyncio.sleep(dt)
		print(alphabot.isClose(), alphabot.getSpeed())        
		if alphabot.isClose() and alphabot.getSpeed() > 0:
			# set speed here and not in task to avoid multiple tasks
			alphabot.setSpeed(-20)
			loop.create_task(avoidObstacle())
		alphabot.update()


#@asyncio.coroutine
def avoidObstacle():
	print("avoidObstacle")

	yield from asyncio.sleep(4)

	alphabot.setTurnSpeed(0, 20)
	yield from asyncio.sleep(3)

	alphabot.setSpeed(20)


@asyncio.coroutine
def speedLoop():

    speed = 20
    yield from asyncio.sleep(2)
    
    while True:
        
        alphabot.setSpeed(speed)
        break
        
        yield from asyncio.sleep(5)
        
        alphabot.setSpeed(-speed)
        yield from asyncio.sleep(5)


if __name__ == '__main__':
    
    try:

        #testServo(Servo1Pin, -30, +30, 5)
        #testServo(Servo2Pin, -90, +90)
        #testMotor()
        #testMotorCounter()
        #testProximity()
                
        import argparse

        parser = argparse.ArgumentParser()
        parser.add_argument('speed', type=float)
        args = parser.parse_args()
        print(args.speed)
        #testSpeed(args.speed)

        left.setSpeed(0)
        right.setSpeed(0)

        tStart = time.time()
        
        loop = asyncio.get_event_loop()
        tasks = [
            asyncio.Task(controlLoop(0.050)),
            asyncio.Task(speedLoop())
            ]
        loop.run_until_complete(asyncio.wait(tasks))
        loop.close()

    finally:
        loop.stop()
        loop.close()
        left.motor.stop()
        right.motor.stop()
        pi.stop()

