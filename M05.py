import Commands

def m05():
  return [
    # Drive from angled jig
    Commands.GyroDrive(speed=-175, distance=910, angle=1),
    # Turn around
    Commands.GyroPivot(angle=143),
    # Line up
    Commands.GyroDrive(speed=-175, distance=90),
    # Turn To LineSquare
    Commands.GyroPivot(angle=-90),
    # LineSquare
    Commands.LineSquare(approachSpeed=75, returnTime=4000),
    # Code for M13 bump
    # Commands.ActMotorTime(motor=2, speed=300, time=850),
    # Commands.GyroPivot(speed=250, angle=-58),
    # Commands.GyroPivot(angle=58),
    # Commands.ActMotorTime(motor=2, speed=-300, time=1000),
    # Turn away
    Commands.GyroPivot(speed=200, angle=-90),
    # Drop Elephent Trunk
    Commands.ActMotorTime(motor=2, speed=300, time=1300, wait=False),
    # Drive to seabed
    Commands.GyroDrive(speed=-175, distance=129),
    # Lift sample
    Commands.ActMotorTime(motor=2, speed=-300, time=2400),
    # Drive away
    Commands.GyroDrive(speed=-200, distance=265),
    # One wheel turn to home
    Commands.DriveMotor(motor="left", angle=160, speed=175),
    # Drive to home
    Commands.DriveMM(speed=250, distance=750),
  ]