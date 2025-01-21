import Commands

def m05():
  return [
    # Drive from angled jig
    Commands.GyroDrive(speed=-175, distance=902, angle=1),
    # Turn around
    Commands.GyroPivot(angle=144),
    # Line up
    Commands.GyroDrive(speed=-175, distance=90),
    # Turn To LineSquare
    Commands.GyroPivot(angle=-90),
    # LineSquare
    Commands.LineSquare(approachSpeed=75, returnTime=4000),
    # M13 bump
    Commands.ActMotorTime(speed=500, time=1000),
    Commands.ActMotorTime(speed=-400, time=1800),
    # Turn away and drop arm
    Commands.GyroPivot(speed=200, angle=-90),
    Commands.ShiftGear(gear=2),
    Commands.ActMotorTime(speed=-350, time=1300, wait=False),
    # Drive to seabed
    Commands.GyroDrive(speed=-175, distance=148),
    # Lift sample
    Commands.ActMotorTime(speed=350, time=2400),
    # Drive away
    Commands.GyroDrive(speed=-200, distance=270),
    # One wheel turn to home
    Commands.DriveMotor(motor="left", angle=167, speed=175),
    # Drive to home
    Commands.DriveMM(speed=250, distance=777),
  ]