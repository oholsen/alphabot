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























