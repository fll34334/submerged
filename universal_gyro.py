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
    possible_values = []
    # POSITIVE NUMBERS
    if target_angle > 0:
      if self.__in_range(0, 360, target_angle):
        print(target_angle, '<--')
        return target_angle

      for x in range(1, 360):
        if target_angle % x == 0:
          possible_values.append(target_angle / x)

      for value in possible_values:
        if target_angle - value == 360:
          print(value, '<--')
          return value
    
    # NEGATIVE NUMBERS
    elif target_angle < 0:
      if self.__in_range(-360, 0, target_angle):
        print(target_angle + 360, '<--')
        return target_angle + 360
    
      for x in range(1, 360):
        if target_angle % x == 0:
          possible_values.append(target_angle / x)

      for value in possible_values:
        if target_angle - value == -360:
          print(value + 360, '<--')
          return value + 360

  def heading(self):
    return self.heading

  def angle(self):
    return self.angle

  def update_angle(self, new_angle):
    self.angle = new_angle
    return self.angle

  def update_heading(self, new_heading):
    self.heading = self.find_nearest_heading(self.angle)
    return self.heading  