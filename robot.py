from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.nxtdevices import LightSensor
from pybricks.media.ev3dev import Image, ImageFile

from pid import PIDController
import math

CurrentGear = 1

class Generic_Robot:
  def __init__(self, EV3Brick, DriveBase, LeftMotor, RightMotor, LightSensorLeft, LightSensorRight, GyroSensor, WheelDiameter, WheelBase):
    self.ev3 = EV3Brick
    self.robot = DriveBase
    self.lm = LeftMotor
    self.rm = RightMotor
    self.sen1 = LightSensorLeft
    self.sen2 = LightSensorRight
    self.gyro = GyroSensor
    self.stopWatch = StopWatch()
    self.left_color_percent = 5
    self.right_color_percent = 5
    self.left_color_black = 60
    self.right_color_black = 60 #guessing which is which, but valuse are from robot
    self.wheel_diameter = WheelDiameter
    self.wheel_base = WheelBase

  ### DRIVE MILIMETERS ###
  def drive_mm(self, angle, speed, mm, rate=500, brake=True):
    self.robot.reset()
    if angle == 0:
      self.robot.settings(speed, rate)
      self.robot.straight(mm)
    else:
      # Check for Backwards & adjust accordingly
      if mm < 0: #works only for distance but not speed
        speed = 0 - speed
        self.robot.drive(speed, angle)
        while self.robot.distance() > mm:
          wait(0)
      else:
        self.robot.drive(speed, angle)
        while self.robot.distance() < mm:
          wait(0)
    self.robot.stop()
    self.lm.brake()
    self.rm.brake()
  
  ### DRIVE TANK ###
  def drive_tank(self, motor_degrees, left_speed, right_speed):
    self.lm.run_angle(left_speed, motor_degrees, then=Stop.BRAKE, wait=False)
    self.rm.run_angle(right_speed, motor_degrees, then=Stop.BRAKE)
  
  ### PIVOT ANGLE ###
  def pivot(self, target_angle, speed):
    left_speed = speed
    right_speed = 0 - speed
    if 0 > target_angle:
      left_speed = 0 - speed
      right_speed = speed
      target_angle = 0 - target_angle

    wheel_circumference_mm = math.pi * self.wheel_diameter
    wheelbase_circumference_mm = math.pi * self.wheel_base
    target_distance = (wheelbase_circumference_mm / 360.0) * target_angle
    motor_degrees = (360 / wheel_circumference_mm) * target_distance

    self.drive_tank(motor_degrees, left_speed, right_speed)
  ### Gyro Pivot ###
  def GyroPivot(self, target_angle, speed):
    self.robot.reset()
    # Reverse logic
    net_angle = target_angle
    left_speed = speed
    right_speed = 0 - speed
    Reverse_angle = False
    if 0 < speed and 0 > target_angle:
      left_speed = left_speed * -1
      right_speed = right_speed * -1
      Reverse_angle=True
    if 0 > speed and 0 < target_angle:
      net_angle = target_angle * -1
      Reverse_angle = True
    if 0 > speed and 0 > target_angle:
      net_angle = target_angle * -1
      left_speed = left_speed * -1
      right_speed = right_speed * -1
    
    # Start moving
    self.gyro.reset_angle(0)
    self.lm.run(speed=left_speed)
    self.rm.run(speed=right_speed)

    # Gyro logic
    if Reverse_angle:
      while self.gyro.angle() > net_angle:
        wait(0)
    else:
      while self.gyro.angle() < net_angle:
        wait(0)

    # Stop
    self.lm.stop()
    self.rm.stop()
    
    return net_angle
  

  ### Gyro Pivot Error Correction ###
  def GyroPivotEC(self, target_angle, speed):
    # Runs Gyro pivot and gets data
    net_angle=self.GyroPivot(target_angle, speed)
    Turn_Error = self.gyro.angle() - net_angle
    print(Turn_Error)
    Pre_angle = self.gyro.angle()
    
    # skiping logic
    if Turn_Error == 0:
      print("skiping")
    else:
      loopcount = 0
      while loopcount <= 4 and not (-1 <= Turn_Error <= 1): 
        self.GyroPivot(Turn_Error, speed=100)
        print("after correction error is:", Pre_angle + self.gyro.angle() - net_angle, "Pre:", Pre_angle, "Current:", self.gyro.angle())
        Turn_Error = self.gyro.angle() - net_angle
        loopcount =+ 1
        print("Looped", loopcount, "times")
    print("Error correction complete with", loopcount, "loops, and a final error of:", Turn_Error)

  ### Gyro Drive Error Correction ###
  def GyroDriveEC(self, distance_mm, speed):
    # Runs GyroDrive and gets data
    self.GyroDrive(distance_mm, speed)
    Pre_angle = self.gyro.angle()
    Turn_Error = self.gyro.angle()
    print(Turn_Error)
    
    # skiping logic
    if Turn_Error == 0:
      print("skiping")
    else:
      loopcount = 0
      while loopcount <= 4 and not (-1 <= Turn_Error <= 1): 
        self.GyroPivot(Turn_Error, speed=100)
        print("after correction error is:", Pre_angle - self.gyro.angle(), "Current, (GyroPivot error):", self.gyro.angle())
        Turn_Error = self.gyro.angle()
        loopcount =+ 1
        print("Looped", loopcount, "times")
    print("Error correction complete with", loopcount, "loops, and a final error of:", Turn_Error + Pre_angle)

  ### DRIVE GYRO MILIMETERS ###
  def gyro_drive(self, angle, speed, distance_mm, gainP=3.719, gainI=0.54, gainD=0.1125, reset_sensor=True):
    self.robot.reset()
    if reset_sensor == True:
      self.gyro.reset_angle(0)
    
    # Check & Adjust for Backwards
    if distance_mm < 0:
      gainP = 0 - gainP
      gainI = 0 - gainI
      gainD = 0 - gainD
      speed = 0 - speed
    if speed < 0:
      gainP = 0 - gainP
      gainI = 0 - gainI
      gainD = 0 - gainD
      speed = 0 - speed
    
      pid_controller = PIDController(gainP, gainI, gainD)
      while self.robot.distance() > distance_mm:
        self.robot.drive(speed, pid_controller.adjust(angle - self.gyro.angle()))
    else:
      pid_controller = PIDController(gainP, gainI, gainD)
      while self.robot.distance() < distance_mm:
        self.robot.drive(speed, pid_controller.adjust(angle - self.gyro.angle()))

    self.robot.stop()
    self.lm.brake()
    self.rm.brake()


  ### CALIBRATE COLOR ###
  def calibrate_color(self):
    count = 0
    while self.ev3.buttons.pressed() != [Button.CENTER] and self.ev3.buttons.pressed() != [Button.DOWN]:
      if count % 1050 == 0:
        self.ev3.screen.draw_image(0, 0, "IMAGES/white-lq-1")
      if count % 3050 == 0:
        self.ev3.screen.draw_image(0, 0, "IMAGES/white-lq-2")
      count += 1
      if count == 3050:
        count = 0
    left_white = self.sen1.reflection()
    right_white = self.sen2.reflection()

    wait(500)

    count = 0
    while self.ev3.buttons.pressed() != [Button.CENTER] and self.ev3.buttons.pressed() != [Button.DOWN]:
      if count % 1050 == 0:
        self.ev3.screen.draw_image(0, 0, "IMAGES/black-lq-1")
      if count % 3050 == 0:
        self.ev3.screen.draw_image(0, 0, "IMAGES/black-lq-2")
      count += 1
      if count == 3050:
        count = 0
    self.left_color_black = self.sen1.reflection()
    self.right_color_black = self.sen2.reflection()

    left_color_range = left_white - self.left_color_black
    right_color_range = right_white - self.right_color_black
    self.left_color_percent = left_color_range / 100.0    #This is 1 percent value (Essentially black)
    self.right_color_percent = right_color_range / 100.0
    self.ev3.screen.clear()

    wait(500)

  def ajustReading(self, reading, leftOrRight):
      if leftOrRight == "Left":
        return reading - self.left_color_black
      else:
        return reading - self.right_color_black

  ### ULTIMATE LINE SQUARING ###
  def black_line_square(self, targetFast, targetBlack, targetWhite, approachSpeed, finetuneSpeed, returnTime):
    def waitUntil(sensor, input1, direction):
      while self.ajustReading(sensor.reflection(), direction) <= input1:
        wait(0)

    def colorIsInRange(value):
      if targetBlack > value and targetWhite < value:
        return True
      else:
        return False

    # Initiate Variables
    self.ev3.light.on(Color.RED)
    reverseFinetuneSpeed = (0 - finetuneSpeed)
    self.stopWatch.pause()
    self.stopWatch.reset()
    self.stopWatch.resume()
    targetFineTune = range(40, 45)
    # Roughly lines up with the black line
    self.robot.drive(approachSpeed, 0)
    while True:
      ref1 = self.ajustReading(self.sen1.reflection(), "Right")
      ref2 = self.ajustReading(self.sen2.reflection(), "Left")

      self.robot.drive(approachSpeed, 0)

      if ref1 <= targetFast:
        self.robot.stop()
        self.lm.hold()
        self.rm.run(approachSpeed)
        waitUntil(self.sen2, targetFast, "Right")
        self.rm.hold()
        break
      elif ref2 <= targetFast:
        self.robot.stop()
        self.rm.hold()
        self.lm.run(approachSpeed)
        waitUntil(self.sen1, targetFast, "Left")
        self.lm.hold()
        break
    # Sets status light to Yellow for fine tuning
    self.ev3.light.on(Color.YELLOW)
    while ref1 not in targetFineTune or ref2 not in targetFineTune:
      ref1 = self.ajustReading(self.sen1.reflection(), "Left")
      ref2 = self.ajustReading(self.sen2.reflection(), "Right")

      # Fine tunes the left motor
      if ref1 in targetFineTune:
        self.lm.hold()
      elif ref1 > max(targetFineTune):
        self.lm.run(reverseFinetuneSpeed)
      else:
        self.lm.run(finetuneSpeed)
      # Fine tunes the right motor
      if ref2 in targetFineTune:
        self.rm.hold()
      elif ref2 > max(targetFineTune):
        self.rm.run(reverseFinetuneSpeed)
      else:
        self.rm.run(finetuneSpeed)

      if self.stopWatch.time() >= returnTime:
        break
    self.lm.hold()
    self.rm.hold()
    self.ev3.light.on(Color.GREEN)


class Robot_Plus(Generic_Robot):
  def __init__(self, name = "Bilbo"):

    ############################################
    ### THIS MUST BE CONFIGURED TO THE ROBOT ###             
    ############################################

    self.wheel_diameter = 62.4
    self.wheel_base = 109.5
    self.name = name
    self.ev3 = EV3Brick()
    self.left_motor = Motor(Port.B, Direction.CLOCKWISE, gears=None)
    self.right_motor = Motor(Port.C, Direction.CLOCKWISE, gears=None)
    self.act_right = Motor(Port.D, Direction.CLOCKWISE, gears=None)
    self.act_left = Motor(Port.A, Direction.CLOCKWISE, gears=None)
    self.drive_base = DriveBase(self.left_motor, self.right_motor, self.wheel_diameter, self.wheel_base)
    #self.infared = InfraredSensor(Port.S1)
    self.left_color = ColorSensor(Port.S2)
    self.right_color = ColorSensor(Port.S3)
    self.gyro_sensor = GyroSensor(Port.S4)

    Generic_Robot.__init__(self, self.ev3, self.drive_base, self.left_motor, self.right_motor, self.left_color, self.right_color, self.gyro_sensor, self.wheel_diameter, self.wheel_base)
  
  def query(self):
    print(self.name)
    return self.name

  def wait(self, time):
    wait(time)

  def reset_motors(self):
    self.act_left.run_angle(1, 1, then=Stop.BRAKE)
    self.act_right.run_angle(1, 1, then=Stop.BRAKE)
    self.left_motor.run_angle(1, 1, then=Stop.BRAKE)
    self.right_motor.run_angle(1, 1, then=Stop.BRAKE)

  def act_run_time(self, motor, speed, time, wait):
    if motor == "left": # <- Left Motor
      self.act_left.run_time(speed, time, wait=wait)
    elif motor == "right": # <- Right Motor
      self.act_right.run_time(speed, time, wait=wait)
  def act_run_angle(self, motor, angle, speed, wait):
    if motor == "left": # <- Left Motor
      self.act_left.run_angle(speed, angle, wait=wait)
    elif motor == "right": # <- Right Motor
      self.act_right.run_angle(speed, angle, wait=wait)

  def drive_motor_angle(self, motor, speed, angle, wait):
    if motor == "left":
      self.left_motor.run_angle(speed, angle, wait=wait)
    elif motor == "right":
      self.right_motor.run_angle(speed, angle, wait=wait)



  def ShiftGear(self, speed, gear):
    self.speed = speed
    global CurrentGear
    if gear > 4 or gear < 1:
      raise ValueError("gear must be between 1 and 4")
    Change = gear - CurrentGear

    if Change == 0:
      wait(0)
    elif Change == 1:
      self.act_run_angle(motor="right", speed=self.speed, angle=30, wait=True)
    elif Change == 2:
      self.act_run_angle(motor="right", speed=self.speed, angle=60, wait=True)
    elif Change == 3:
      self.act_run_angle(motor="right", speed=self.speed, angle=90, wait=True)
    elif Change == 4:
      self.act_run_angle(motor="right", speed=self.speed, angle=-30, wait=True)
    elif Change == -1:
      self.act_run_angle(motor="right", speed=self.speed, angle=-30, wait=True)
    elif Change == -2:
      self.act_run_angle(motor="right", speed=self.speed, angle=-60, wait=True)
    elif Change == -3:
      self.act_run_angle(motor="right", speed=self.speed, angle=-90, wait=True)
    elif Change == -4:
      self.act_run_angle(motor="right", speed=self.speed, angle=30, wait=True)
    CurrentGear = gear