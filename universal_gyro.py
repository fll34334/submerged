import math

class Universal_Gyro:
  def __init__(self):
    self.heading = 0
    self.angle = 0

  def __in_range(self, low, high, target):
    if target >= low and target <= high:
      return True
    else:
      return False

  def find_nearest_heading(self, target_angle):
    return target_angle % 360

  def heading(self):
    return self.heading

  def angle(self):
    return self.angle

  def update(self, angle):
    self.angle = angle
    self.heading = self.find_nearest_heading(self.angle)
    return self.angle, self.heading