import Commands

def m02():
  return [
    Commands.DriveMM(speed=100, distance=-400),
    Commands.GyroPivot(angle=47),
    Commands.DriveMM(speed=150, distance=-125),
  ]