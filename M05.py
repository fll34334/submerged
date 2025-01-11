import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-175, distance=50),
    Commands.GyroPivot(angle=-55),
    Commands.GyroDrive(speed=-175, distance=490),
    Commands.GyroPivot(angle=6),
    Commands.GyroDrive(speed=-175, distance=400),
    Commands.ActMotorTime(motor=2, speed=200, time=1800),
    Commands.ActMotorTime(motor=2, speed=-200, time=1800),
  ]