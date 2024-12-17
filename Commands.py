from robot import Robot_Plus

class GyroDrive:
  def __init__(self, angle=0, speed=200, distance=0, reset_sensor=True):
    self.angle = angle
    self.speed = speed
    self.distance_mm = distance
    self.reset_sensor = reset_sensor

  def run(self, robot):
    robot.gyro_drive(self.angle, self.speed, self.distance_mm, reset_sensor=self.reset_sensor)

class DriveMM:
  def __init__(self, angle=0, speed=200, distance=0, rate=500, brake=True):
    self.angle = angle
    self.speed = speed
    self.distance_mm = distance
    self.rate = rate
    self.brake = brake

  def run(self, robot):
    robot.drive_mm(self.angle, self.speed, self.distance_mm, self.rate, self.brake)

class Pivot:
  def __init__(self, angle=0, speed=100, wait=75):
    self.angle = angle
    self.speed = speed
    self.wait = wait

  def run(self, robot):
    robot.wait(self.wait)
    robot.pivot(self.angle, self.speed)
    robot.wait(self.wait)

class LineSquare:
  def __init__(self, target=20, targetBlack=13, targetWhite=80, approachSpeed=100, finetuneSpeed=50, returnTime=2500):
    self.target = target
    self.targetBlack = targetBlack
    self.targetWhite = targetWhite
    self.approachSpeed = approachSpeed
    self.finetuneSpeed = finetuneSpeed
    self.returnTime = returnTime

  def run(self, robot):
    robot.black_line_square(self.target, self.targetBlack, self.targetWhite, self.approachSpeed, self.finetuneSpeed, self.returnTime)

class DriveMotor:
  def __init__(self, motor="left", angle=0, speed=200, wait=True):
    self.motor = motor
    self.angle = angle
    self.speed = speed
    self.wait = wait

  def run(self, robot):
    robot.drive_motor_angle(self.motor, self.speed, self.angle, self.wait)

class ActMotorAngle:
  def __init__(self, motor="left", angle=0, speed=200, wait=True):
    self.motor = motor
    self.angle = angle
    self.speed = speed
    self.wait = wait

  def run(self, robot):
    robot.act_run_angle(self.motor, self.angle, self.speed, self.wait)

class ActMotorTime:
  def __init__(self, motor="left", time=1000, speed=200, wait=True):
    self.motor = motor
    self.time = time
    self.speed = speed
    self.wait = wait

  def run(self, robot):
    robot.act_run_time(self.motor, self.speed, self.time, self.wait)

class Wait:
  def __init__(self, time=1000):
    self.time = time

  def run(self, robot):
    robot.wait(self.time)

class ShiftGear:
  def _init_(self, speed=150, gear=1):
    self.speed = speed
    self.gear = gear
    #gear = self.gear
    #CurrentGear = 1 # can it be stored??
    

  def run(self, robot):
    self.gear = 1
    gear = self.gear
    Change = gear - CurrentGear
    def gearlogic(CurrentGear, gear, Change):
      print(CurrentGear, " to ", self.gear, " = ")
      if CurrentGear + Change > 4:
        CurrentGear = CurrentGear + Change - 4
      elif CurrentGear + Change < 1:
        CurrentGear = CurrentGear + Change + 4
      elif 1 < CurrentGear + Change <= 4:
        CurrentGear =+ Change
      print(CurrentGear)
    if Change == 0:
      robot.wait(time=100)
    elif Change == 1:
      robot.act_run_angle("left", self.speed, angle=40)
      gearlogic()
    elif Change == 2:
      robot.act_run_angle("left", self.speed, angle=80)
      gearlogic()
    elif Change == 3:
      robot.act_run_angle("left", self.speed, angle=120)
      gearlogic()
    elif Change == 4:
      robot.act_run_angle("left", self.speed, angle=-40)
      gearlogic()
    elif Change == -1:
      robot.act_run_angle("left", self.speed, angle=-40)
      gearlogic()
    elif Change == -2:
      robot.act_run_angle("left", self.speed, angle=-80)
      gearlogic()
    elif Change == -3:
      robot.act_run_angle("left", self.speed, angle=-120)
      gearlogic()
    elif Change == -4:
      robot.act_run_angle("left", self.speed, angle=40)
      gearlogic()