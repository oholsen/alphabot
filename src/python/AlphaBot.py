import asyncio
import json
import logging
import time

import pigpio
from PID import PID

log = logging.getLogger(__name__)
pi = pigpio.pi()

CNTL = 7  # CE1
CNTR = 8  # CE0

MLIN1 = 12
MLIN2 = 13
MLEN = 6

MRIN1 = 20
MRIN2 = 21
MREN = 26

ServoTiltPin = 27  # tilt neg up
ServoPanPin = 22  # pan neg left

# Infrared detectors
DR = 16
DL = 19


class Proximity(object):

    def __init__(self, pin):
        self.pin = pin
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        # self.cb = pi.callback(pin, pigpio.RISING_EDGE) #, cb)

    def isClose(self):
        return pi.read(self.pin) == 0


proximityLeft = Proximity(DL)
proximityRight = Proximity(DR)


def clip2(x, lo, hi):
    return min(max(x, lo), hi)


class Servo(object):

    def __init__(self, pin, angle: float = 0, angleRange: float = 60, pulseWidthRange: float = 500):
        self.angleRange = float(angleRange)
        self.pulseWidthRange = pulseWidthRange
        self.pin = pin
        pw = self._pulseWidth(angle)
        print("pulse width", self.pin, pw)
        pi.set_servo_pulsewidth(self.pin, pw)

    def _pulseWidth(self, angle: float):
        # neutral is 1.5ms pulse
        # +-0.5ms is 90 degrees, depending on servo
        return 1500 + self.pulseWidthRange * clip2(angle / self.angleRange, -1, 1)

    def setAngle(self, angle):
        pw = self._pulseWidth(angle)
        print("pulse width", self.pin, pw)
        pi.set_servo_pulsewidth(self.pin, pw)

    def stop(self):
        print("pulse width", self.pin, 0)
        pi.set_servo_pulsewidth(self.pin, 0)


class Motor(object):

    def __init__(self, in1, in2, en):
        self.IN1 = in1
        self.IN2 = in2
        self.EN = en

        pi.set_mode(self.IN1, pigpio.OUTPUT)
        pi.set_mode(self.IN2, pigpio.OUTPUT)
        pi.set_mode(self.EN, pigpio.OUTPUT)

        self.power = 0
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
        self.power = power
        pi.set_PWM_dutycycle(self.EN, min(255, int(255.0 * abs(power) / 100)))


"""
def cb(gpio, level, tick):
        sys.stdout.write('!')
        sys.stdout.flush()
"""


class Counter(object):
    """Cannot detect slow speeds, depending on integration/count time"""

    def __init__(self, pin):
        self.cb = pi.callback(pin, pigpio.RISING_EDGE)  # , cb)
        # self.lastTime = time.time()
        # self.lastCount = 0
        self.alpha = 0.9
        self._speed = 0
        self.counts = [(time.time(), 0)]
        self.n = 4

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

        dcount = count - clast
        dt = t - tlast
        v = float(dcount) / dt
        self._speed = self.alpha * v + (1 - self.alpha) * self._speed
        return self._speed

    def count(self):
        return self.cb.tally()


class MotorController:

    def __init__(self, name, motor, counter):
        self.name = name
        self.motor = motor
        self.counter = counter
        self.pid = PID(0.5, 0.3, 0, I_max=50, I_min=-50)

    def update(self, t: float, debug=False):

        # print("update", self.name, self.pid.getPoint())
        countRate = self.counter.speed()
        # print self.counter.speed(), countRate, self.counter.times

        # Don't update PID when the motor is stopped
        if abs(self.pid.getPoint()) < 1:
            self.motor.setPower(0)
            self.pid.reset()
            return

        # TODO: can set direction in counter
        if self.motor.power < 0:
            countRate = -countRate

        power = self.pid.update(t, countRate)
        # print("update power", self.name, power)
        if power is None:
            return

        power = -power
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
        self.pid.setPoint(speed)

    def getSpeed(self):
        return self.pid.getPoint()

    def stop(self):
        self.setSpeed(0)
        self.motor.stop()

    def status(self):
        return {
            "power": self.motor.power,
            "setSpeed": self.getSpeed(),
            "speed": self.counter.speed(),
            "pid": self.pid.status(),
        }


class AlphaBot():

    def __init__(self):
        self.cameraPan = Servo(ServoPanPin)
        self.cameraTilt = Servo(ServoTiltPin)
        self.left = MotorController('L', Motor(MLIN1, MLIN2, MLEN), Counter(CNTL))
        self.right = MotorController('R', Motor(MRIN2, MRIN1, MREN), Counter(CNTR))
        self.speed: float = 0
        self.turnSpeed: float = 0
        self.stopped = True

    def stop(self):
        log.info("stop")
        self.stopped = True
        self.speed = 0
        self.turnSpeed = 0
        self.left.stop()
        self.right.stop()
        self.cameraPan.stop()
        self.cameraTilt.stop()

    def status(self):
        return {
            "stopped": self.stopped,
            # "proxLeft": proximityLeft.isClose(),
            # "proxRight": proximityRight.isClose(),
            "motorLeft": self.left.status(),
            "motorRight": self.right.status(),
            "speed": self.speed,
            "turn": self.turnSpeed,
        }

    def isClose(self):
        return proximityLeft.isClose() or proximityRight.isClose()

    def setSpeed(self, speed: float):
        log.info("speed %g", speed)
        self.speed = speed
        self._command()

    def setTurn(self, turn: float):
        log.info("turn %g", turn)
        self.turnSpeed = turn
        self._command()

    def getSpeed(self) -> float:
        return 0.5 * (self.left.getSpeed() + self.right.getSpeed())

    def setSpeedAndTurn(self, speed: float, turnSpeed: float):
        log.info("speedturn %g %g", speed, turnSpeed)
        self.speed = speed
        self.turnSpeed = turnSpeed
        self._command()

    def _command(self):
        log.debug("_command %g %g", self.speed, self.turnSpeed)
        self.stopped = False
        self.left.setSpeed(self.speed + self.turnSpeed)
        self.right.setSpeed(self.speed - self.turnSpeed)

    def update(self):
        if not self.stopped:
            t = time.time()
            self.left.update(t)
            self.right.update(t)


alphabot = AlphaBot()


async def controlLoop(dt):
    while True:
        await asyncio.sleep(dt)
        try:
            # print(alphabot.isClose(), alphabot.getSpeed())        
            if 0:
                # automatic obstacle avoidance
                if alphabot.isClose() and alphabot.speed > 0:
                    # set speed here and not in task to avoid multiple tasks
                    alphabot.setSpeed(-20)
                    loop.create_task(avoidObstacle(20))
            alphabot.update()
        except:
            log.exception("Control loop")


async def avoidObstacle(speed):
    log.info("avoidObstacle")
    await asyncio.sleep(2.5)
    alphabot.setSpeedAndTurn(0, 20)
    await asyncio.sleep(1.5)
    alphabot.setSpeed(speed)


async def speedLoop(speed):
    await asyncio.sleep(2)
    while True:
        alphabot.setSpeed(speed)
        break
        await asyncio.sleep(5)
        alphabot.setSpeed(-speed)
        await asyncio.sleep(5)


def handle(message):
    try:
        log.info("Handle %r", message)
        cols = message.split()
        cmd = cols.pop(0)
        if cmd == "!" or cmd == ".":
            # ! is reset, . is stop
            alphabot.stop()
        elif cmd == "t":
            alphabot.setSpeed(float(cols[0]))
        elif cmd == "r":
            alphabot.setTurn(float(cols[0]))
        elif cmd == "pan:":
            alphabot.cameraPan.setAngle(float(cols[0]))
        elif cmd == "tilt":
            alphabot.cameraTilt.setAngle(float(cols[0]))
        else:
            log.warning("Ignore message %r", message)
    except:
        log.exception("Handle message %r", message)


clients = set()


async def wscontrol(websocket, path):
    clients.add(websocket)
    try:
        async for message in websocket:
            handle(message)
    except Exception as e:
        log.exception("websocket: %s", e)
    finally:
        clients.remove(websocket)


async def post(msg):
    if clients:
        await asyncio.wait([client.send(msg) for client in clients])


async def status():
    while True:
        await asyncio.sleep(0.2)
        try:
            status = alphabot.status()
            await post(json.dumps(status))
        except Exception as e:
            log.exception("status: %s", e)


if __name__ == '__main__':

    import argparse
    import websockets

    parser = argparse.ArgumentParser()
    # parser.add_argument('speed', type=float)
    args = parser.parse_args()

    logging.basicConfig(level=logging.DEBUG)
    logging.getLogger("websockets").setLevel(logging.INFO)

    # log.info("Speed %f", args.speed)
    # testSpeed(args.speed)

    tStart = time.time()
    loop = asyncio.get_event_loop()

    try:
        # alphabot.setSpeed(args.speed)
        tasks = [
            asyncio.Task(controlLoop(0.050)),
            asyncio.Task(status()),
            # asyncio.Task(speedLoop(args.speed)),
            websockets.serve(wscontrol, '0.0.0.0', 8000)
        ]
        loop.run_until_complete(asyncio.wait(tasks))
        loop.run_forever()
        log.info("Loop complete")
    finally:
        loop.stop()
        loop.close()
        alphabot.stop()
        pi.stop()
