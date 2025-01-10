import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-175, distance=50),
    Commands.GyroPivot(angle=-55),
    Commands.GyroDrive(speed=-175, distance=490),
    Commands.GyroDrive(speed=-175, distance=350, angle=3),
  ]