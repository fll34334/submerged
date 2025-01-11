import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-150, distance=20),
    Commands.GyroPivot(angle=-56),
    Commands.GyroDrive(speed=-175, distance=490),
    #Commands.GyroPivot(angle=3),
    Commands.GyroDrive(speed=-175, distance=400),
    #Commands.GyroPivot(angle=-36),
    # Commands.ActMotorTime(motor=2, speed=200, time=1500),
    # Commands.ActMotorTime(motor=2, speed=-300, time=2500),
  ]