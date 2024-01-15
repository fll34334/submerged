from robot import Robot_Plus

class GyroDrive:
  def __init__(self, angle=0, speed=0, distance=0, reset_sensor=True):
    self.angle = angle
    self.speed = speed
    self.distance_mm = distance
    self.reset_sensor = reset_sensor

  def run(self, robot):
    robot.gyro_drive(self.angle, self.speed, self.distance_mm, reset_sensor=self.reset_sensor)

class DriveMM:
  def __init__(self, angle=0, speed=0, distance=0, rate=500, brake=True):
    self.angle = angle
    self.speed = speed
    self.distance_mm = distance
    self.rate = rate
    self.brake = brake

  def run(self, robot):
    robot.drive_mm(self.angle, self.speed, self.distance_mm, self.rate, self.brake)

class Pivot:
  def __init__(self, angle, speed, wait=75):
    self.angle = angle
    self.speed = speed
    self.wait = wait

  def run(self, robot):
    robot.wait(self.wait)
    robot.pivot(self.angle, self.speed)
    robot.wait(self.wait)

class LineSquare:
  def __init__(self, target, targetBlack, targetWhite, approachSpeed, finetuneSpeed, returnTime=2):
    self.target = target
    self.targetBlack = targetBlack
    self.targetWhite = targetWhite
    self.approachSpeed = approachSpeed
    self.finetuneSpeed = finetuneSpeed
    self.returnTime = returnTime

  def run(self, robot):
    robot.black_line_square(self.target, self.targetBlack, self.targetWhite, self.approachSpeed, self.finetuneSpeed, self.returnTime)

class DriveMotor:
  def __init__(self, motor, angle, speed, wait=True):
    self.motor = motor
    self.angle = angle
    self.speed = speed
    self.wait = wait

  def run(self, robot):
    if self.motor == "left":
      robot.left_motor.run_angle(self.speed, self.angle, self.wait)
    elif self.motor == "right":
      robot.right_motor.run_angle(self.speed, self.angle, self.wait)

class ActMotorAngle:
  def __init__(self, motor, angle, speed, wait=True):
    self.motor = motor
    self.angle = angle
    self.speed = speed
    self.wait = wait

  def run(self, robot):
    robot.act_run_angle(self.motor, self.angle, self.speed, self.wait)

class ActMotorTime:
  def __init__(self, motor, time, speed, wait=True):
    self.motor = motor
    self.time = time
    self.speed = speed
    self.wait = wait

  def run(self, robot):
    robot.act_run_time(self.motor, self.speed, self.time, self.wait)

class Wait:
  def __init__(self, time=0):
    self.time = time

  def run(self, robot):
    robot.wait(self.time)