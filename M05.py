import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-175, distance=90),
    Commands.GyroPivot(angle=-56),
    Commands.GyroDrive(speed=-175, distance=1050),
  ]