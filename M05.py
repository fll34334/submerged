import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-150, distance=30),
    Commands.GyroPivot(angle=-51),
    Commands.GyroDrive(speed=-175, distance=890),
    Commands.GyroPivot(angle=30),
    Commands.GyroDriveREG(speed=-175, distance=-50),
    Commands.GyroPivot(angle=57),
    Commands.GyroDriveREG(speed=-175, distance=80),
    Commands.GyroPivot(angle=-50),
    Commands.GyroDrive(speed=-150, distance=45),
    Commands.LineSquare(approachSpeed=75, returnTime=3500),
    Commands.GyroDrive(speed=-150, distance=25),
    Commands.GyroPivot(speed=250, angle=-65),
    Commands.GyroPivot(angle=65),
    Commands.GyroDrive(speed=-175, distance=-20),
    Commands.GyroPivot(angle=-90),
    Commands.ActMotorTime(motor=2, speed=300, time=1300, wait=False),
    Commands.GyroDrive(speed=-175, distance=120),
    Commands.ActMotorTime(motor=2, speed=-300, time=2400),
    Commands.GyroDrive(speed=-200, distance=300),
  ]