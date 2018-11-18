import RPi.GPIO as GPIO
import time
from PID import PID
import numpy as np


#Ab = AlphaBot()
#Ab.stop()
CNTL = 7 # CE1
CNTR = 8 # CE0

MLIN1 = 12
MLIN2 = 13
MLEN = 6

MRIN1 = 20
MRIN2 = 21
MREN = 26

Servo1Pin = 27
Servo2Pin = 22

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Servo(object):
        def __init__(self, pin, angle = 0, angleRange = 90):
                self.angleRange = angleRange
                GPIO.setup(pin, GPIO.OUT)
                self.pwm = GPIO.PWM(pin, 50) # frequency, 20ms period
                self.pwm.start(self._dutyCycle(angle))

        def _dutyCycle(self, angle):
                # return duty cycle in percent of 20ms period
                # neutral is 1.5ms pulse
                # 0.5ms is 90 degrees
                

                if angle > 0: angle = min(angle, self.angleRange)
                else: angle = max(angle, -self.angleRange)
                print(angle)
                return 100.0 * (1.5 + 0.5 * angle / self.angleRange) / 20
                #return 12.5 - 10.0 * float(angle) / 180
                
        def angle(self, angle):
                self.pwm.ChangeDutyCycle(self._dutyCycle(angle))

                
class Motor(object):

        def __init__(self, in1, in2, en):
                self.IN1 = in1
                self.IN2 = in2
                self.EN = en

                GPIO.setup(self.IN1, GPIO.OUT)
                GPIO.setup(self.IN2, GPIO.OUT)
                GPIO.setup(self.EN, GPIO.OUT)
                #self.forward()
                self.stop()
                self.pwm = GPIO.PWM(self.EN, 500)
                self.pwm.start(0)

        def forward(self):
                GPIO.output(self.IN1,GPIO.HIGH)
                GPIO.output(self.IN2,GPIO.LOW)

        def stop(self):
                GPIO.output(self.IN1,GPIO.LOW)
                GPIO.output(self.IN2,GPIO.LOW)

        def backward(self):
                GPIO.output(self.IN1,GPIO.LOW)
                GPIO.output(self.IN2,GPIO.HIGH)
                
        def setPower(self, power):
                if power >= 0:
                        self.forward()
                else:
                        self.backward()
                self.pwm.ChangeDutyCycle(min(100, abs(power)))

class Counter(object):
        
        def __init__(self, pin):
                self.count = 0
                self.lastTime = None
                self.times = []
                self.n = 4
                GPIO.setup(pin, GPIO.IN)
                GPIO.add_event_detect(pin, GPIO.FALLING, callback=self.countEvent)

        def countEvent(self, pin):
                self.count += 1                
                t = time.time()
                if self.lastTime is not None:
                        self.times.append(t - self.lastTime)
                        self.times = self.times[-self.n:]
                self.lastTime = t

        def speed(self):
                
                # TODO: use last if rate is slow, in order to increase response, use more and more averaging if rate is high
                # add dts back to e.g. 0.1 seconds...
                # TODO: slow speed/stopped, then no ticks, should not use last entry, since it is old...
                # check on lastTime...
                
                if 0:
                        n = 0
                        dt = 0
                        dts = self.times[:]
                        dts.reverse()
                        for d in dts:
                                if n > 0 and dt + d > 0.1: break
                                dt += d
                                n += 1

                        if n == 0: return 0.0
                        return n / dt
                #return 1.0 / self.times[-1]
                if len(self.times) < self.n:
                        return 0.0
                return len(self.times) / sum(self.times)


class MotorCounter:

        def __init__(self, name, motor, counter):
                self.name = name
                self.motor = motor
                self.counter = counter
                self.pid = PID(0.5, 1.4, 0.8, Integrator_max=300, Integrator_min=-300)

                self.lastUpdateTime = time.time()
                self.lastCount = self.counter.count

                
        def update(self):
                t = time.time()
                if 0:
                        count = self.counter.count
                        dcount = count - self.lastCount
                        dt = t - self.lastUpdateTime
                        countRate = dcount / dt
                dcount = 0
                countRate = self.counter.speed()
                #print self.counter.speed(), countRate, self.counter.times

                # TODO: can set direction in counter
                if self.pid.getPoint() < 0:
                        countRate = -countRate
                        
                power = self.pid.update(t, countRate)
                if power is None: return
                print('%s  %6.2f %3d %6.2f  %6.2f  %6.2f  %6.2f' % (self.name, countRate, dcount, self.pid.P, self.pid.I, self.pid.D, power), end=' ') 
                if power > 0:
                        power = min(power, 100)
                else:
                        power = max(power, -100)
                self.motor.setPower(power)
                print(' %6.2f' % power)

                if 0:
                        self.lastCount = count
                        self.lastUpdateTime = t

        def setSpeed(self, speed):
                self.pid.setPoint(time.time(), speed)



if __name__ == '__main__':
        angle = 0
        s = Servo(Servo1Pin, angle)
        while 1:
                time.sleep(0.2)
                #continue
                angle += 5
                if angle > 90:
                        angle = -90
                s.angle(angle)

        import argparse



        parser = argparse.ArgumentParser()
        parser.add_argument('speed', type=float)
        args = parser.parse_args()
        print(args.speed)
        
        left  = MotorCounter('L', Motor(MLIN1, MLIN2, MLEN), Counter(CNTL))
        right = MotorCounter('R', Motor(MRIN2, MRIN1, MREN), Counter(CNTR))

        speed = args.speed

        try:
                left.setSpeed(speed)
                right.setSpeed(speed)

                m = left
                def do(s):
                        print(s)
                        m.motor.setPower(s)
                        for i in range(4):
                                time.sleep(0.5)
                                print(m.counter.count)

                if 0:
                        #speed = 0
                        #Ab.setMotor(speed, speed)
                        #Ab.forward()
                        #time.sleep(2.0)

                        while True:
                                do(-100)
                                do(-50)
                                do(0)
                                do(30)
                                do(50)
                                do(100)

                h = [0] * 30
                
                dt = 0.1
                while True:
                        # TODO: only update PID when new speed is available!?
                        time.sleep(dt)
                        
                        left.update()
                        right.update()
                        #print ' '.join('%.3f' % dt for dt in right.counter.times)
                        #dts = right.counter.times
                        #print np.mean(dts), np.std(dts)

        except:
                GPIO.cleanup()
                raise

