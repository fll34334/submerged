#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.nxtdevices import LightSensor

from robot import Generic_Robot
from robot import Robot_Plus

## deltaTime = 0.0099343s
## Oscillation Period = 0.2s
Kc = 13.5
dT = 9.9343 #ms
Pc = 0.1

robot = Robot_Plus()

# working: robot.gyro_drive(0, 150, 300, 0.60 * Kc, 0, 0)
# working: Zeigler-Nichols MSTAR-labs fast (better than previous)
# working: Zeigler-Nichols MSTAR-labs slow (best)

## NEW OBJECTIVE: TUNE TO 200mm/s
# base: robot.gyro_drive(0, 200, 700, 0, 0, 0)

# Zeigler-Nichols Wikipedia: robot.gyro_drive(0, 200, 700, 4.926, 0.0150, 404.3606)
# Zeigler-Nichols ScieneDirect: robot.gyro_drive(0, 200, 700, 4.926, 1.312, 0.082)
# Zeigler-Nichols MSTAR-labs {fast dT}: robot.gyro_drive(0, 200, 700, 4.926, 0.328, 0.082)
# Zeigler-Nichols MSTAR-labs {slow dT}: robot.gyro_drive(0, 200, 700, 3.719, 0.54, 0.1125)

####### FASTER DELTA TIME

# Kc = 8.21 @ 20hz
# dT = 50ms / 20hz

## Oscillation Tests
# timer = StopWatch()
# timer.pause()
# timer.reset()
# timer.resume()
# robot.gyro_drive(0, 200, 700, 4.926, 0.328, 0.082)
# timer.pause()
# print(timer.time())

# oscillated 6 times within 3686ms 1
# oscillated 6 times within 3690ms 2
# oscillated 5 times within 3663ms 3
# oscillated 6 times within 3655ms 4
# oscillated 5 times within 3687ms 5
# oscillated 5 times within 3679ms 6
# oscillated 6 times within 3704ms 7
# oscillated 6 times within 3669ms 8
# oscillated 6 times within 3679ms 9
# oscillated 5 times within 3663ms 10

# avg runtime = 3677.5 over 700mm
# avg oscillations per run = 5.6 over 700mm
# Pc = 656.6964285714287ms

#robot.gyro_drive(0, 200, 700, 4.926, 0.328, 0.082)



####### SLOWER DELTA TIME

# Kc = 6.2 @ 40hz
# dT = 100ms / 40hz

## Oscillation Tests
# timer = StopWatch()
# timer.pause()
# timer.reset()
# timer.resume()
# robot.gyro_drive(0, 200, 700, 6.35, 0, 0)
# timer.pause()
# print(timer.time())

# oscillated 4 times within 3789ms 1
# oscillated 4 times within 3788ms 2
# oscillated 4 times within 3811ms 3
# oscillated 4 times within 3777ms 4
# oscillated 4 times within 3768ms 5
# oscillated 5 times within 3792ms 6
# oscillated 4 times within 3776ms 7
# oscillated 4 times within 3780ms 8
# oscillated 4 times within 3784ms 9
# oscillated 5 times within 3765ms 10

# avg runtime = 3783ms over 700mm
# avg oscillations per run = 4.2 over 700mm
# Pc = 900.7142857142857ms



# old robot.GyroDrive(0, 200, 700, 3.719, 0.54, 0.1125)

robot.GyroDrive(0, 200, 1000, 2.8, 0.54, 0.1125)





#test 1 gyro drive: P=0, I=0, D=0. deviated: 25mm
#test 2 gyro drive: P=1, I=0, D=0. deviated: 100mm
#test 3 gyro drive: P=3, I=0, D=0. deviated: 15mm
#test 4 gyro drive: P=5, I=0, D=0. deviated: 15mm
#test 5 gyro drive: P=6, I=0, D=0. deviated: 7mm
#test 6 gyro drive: P=6, I=0, D=1. deviated: 13mm
#test 7 gyro drive: P=6, I=0, D=2. deviated: 1mm
#test 8 gyro drive: P=6, I=0, D=1.5. deviated: 10mm
#test 9 gyro drive: P=6, I=0, D=2. deviated: 45mm
#test 10 gyro drive: P=6, I=0, D=3. deviated: 10mm
#test 11 gyro drive: P=6, I=0, D=4. deviated: 1mm
#test 12 gyro drive: P=5, I=0, D=4. deviated: 30mm
#test 13 gyro drive: P=5, I=0, D=3. deviated: 1mm
#test 14 gyro drive: P=2.5, I=0, D=1.5. deviated: 38mm
#test 15 gyro drive: P=2.5, I=0, D=1.5. deviated: 27mm
#test 16 gyro drive: P=1.5, I=0, D=1.2. deviated: 23mm (last years values)
#test 17 gyro drive: P=3, I=0, D=1. deviated: CIRCLESmm (gyro reading 0-300 3s)
#test 18 gyro drive: P=3, I=0, D=1. deviated: 45mm (fixed circle bug)
#test 19 gyro drive: P=5, I=0, D=3. deviated: 70mm
## End Test. ending vaules: P=5,I=0,D=3

## Day 2
## total dist = 1200mm
#test 1 gyro drive: P=0, I=0, D=0 deviated: 60mm, traveled dist: 1185mm
#test 2 gyro drive: P=1, I=0, D=0 deviated: 40mm, traveled dist: 1180mm
#test 3 gyro drive: P=2, I=0, D= 0deviated: 15mm, traveled dist: 1185mm
#test 4 gyro drive: P=4, I=0, D=0 deviated: 35mm, traveled dist: 1180mm
#test 5 gyro drive: P=8, I=0, D=0 deviated: 15mm, traveled dist: 1190mm
#test 6 gyro drive: P=6, I=0, D=0 deviated: 35mm, traveled dist: 1180mm
#test 7 gyro drive: P=6, I=0.1, D=0 deviated: 30mm, traveled dist: 1180mm
#test 8 gyro drive: P=6, I=0.2, D=0 deviated: 40mm, traveled dist: 1165mm
#test 9 gyro drive: P=6, I=0.15, D=0 deviated: 35mm, traveled dist: 1180mm
#test 10 gyro drive: P=6, I=0.15, D=1 deviated: 35mm, traveled dist: 1185mm
#test 11 gyro drive: P=6, I=0.15, D=2 deviated: 35mm, traveled dist: 1185mm
#test 12 gyro drive: P=6, I=0.15, D=1.5 deviated: 15mm, traveled dist: 1185mm
#test x gyro drive: P=, I=, D= deviated: mm, traveled dist: mm
## End Test. ending values: P=6,I=0.15,D=1.5