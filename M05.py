import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-150, distance=30),
    Commands.GyroPivot(angle=-51),
    Commands.GyroDrive(speed=-175, distance=490),
    Commands.GyroDrive(speed=-175, distance=400),
    Commands.ActMotorTime(motor=2, speed=200, time=1500, wait=False),
    Commands.GyroDrive(speed=-175, distance=90),
    Commands.GyroPivot(angle=-36)
    Commands.ActMotorTime(motor=2, speed=-300, time=2500),
  ]