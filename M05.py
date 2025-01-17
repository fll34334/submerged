import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-150, distance=30),
    Commands.GyroPivot(angle=-51),
    Commands.GyroDrive(speed=-175, distance=910),
    Commands.GyroPivot(angle=143),
    Commands.GyroDrive(speed=-175, distance=130),
    Commands.GyroPivot(angle=-90),
    Commands.GyroDrive(speed=-150, distance=20),
    Commands.LineSquare(approachSpeed=75, returnTime=3500),
    Commands.ActMotorTime(motor=2, speed=300, time=600),
    Commands.GyroPivot(speed=200, angle=-57),
    Commands.GyroPivot(speed=200, angle=57),
    Commands.ActMotorTime(motor=2, speed=-300, time=1000),
    Commands.GyroPivot(speed=200, angle=-90),
    Commands.ActMotorTime(motor=2, speed=300, time=1300, wait=False),
    Commands.GyroDrive(speed=-175, distance=136),
    Commands.ActMotorTime(motor=2, speed=-300, time=2400),
    Commands.GyroDrive(speed=-200, distance=250),
  ]