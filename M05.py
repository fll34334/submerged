import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-175, distance=90),
    Commands.GyroPivot(angle=-57),
    Commands.GyroDriveREG(speed=-150, distance=750),
  ]