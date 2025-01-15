import Commands

def m05():
  return [
    Commands.GyroDrive(speed=-150, distance=30),
    Commands.GyroPivot(angle=-51),
    Commands.GyroDrive(speed=-175, distance=490),
    Commands.GyroDrive(speed=-175, distance=400),
    Commands.GyroPivot(angle=30),
    Commands.GyroDriveREG(speed=-175, distance=-50),
    Commands.GyroPivot(angle=57),
    Commands.GyroDrive(speed=-175, distance=80),
    Commands.GyroPivot(angle=-35),
    Commands.GyroDrive(speed=-150, distance=38),
    Commands.LineSquare(returnTime=3000),
    Commands.ActMotorTime(motor=2, speed=300, time=700),
    Commands.GyroPivot(speed=200, angle=-65),
    Commands.GyroPivot(angle=65),
    Commands.GyroDrive(speed=-175, distance=-20),
    Commands.GyroPivot(angle=-90),
    Commands.ActMotorTime(motor=2, speed=300, time=500, wait=False),
    Commands.GyroDrive(speed=-175, distance=100),
    Commands.ActMotorTime(motor=2, speed=-300, time=2500),
  ]