import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-175, distance=910, angle=1),
    Commands.GyroPivot(angle=143),
    Commands.GyroDrive(speed=-175, distance=90),
    Commands.GyroPivot(angle=-90),
    Commands.LineSquare(approachSpeed=75, returnTime=4000),
    #for bump
    # Commands.ActMotorTime(motor=2, speed=300, time=850),
    # Commands.GyroPivot(speed=250, angle=-58),
    # Commands.GyroPivot(angle=58),
    # Commands.ActMotorTime(motor=2, speed=-300, time=1000),
    Commands.GyroPivot(speed=200, angle=-90),
    Commands.ActMotorTime(motor=2, speed=300, time=1300, wait=False),
    Commands.GyroDrive(speed=-175, distance=129),
    Commands.ActMotorTime(motor=2, speed=-300, time=2400),
    Commands.GyroDrive(speed=-200, distance=265),
    Commands.DriveMotor(motor="left", angle=160, speed=175),
    Commands.DriveMM(speed=250, distance=750),
  ]