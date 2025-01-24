import Commands

def m05():
  return [
    # Drive from angled jig
    Commands.GyroDrive(speed=-175, distance=870),
    # Turn around
    Commands.GyroPivot(angle=145),
    # Line up
    Commands.GyroDrive(speed=-175, distance=101),
    # Turn To LineSquare
    Commands.GyroPivot(angle=-90),
    # LineSquare
    Commands.LineSquare(approachSpeed=75, returnTime=4000),
    # M13 bump
    # Turn away and drop arm
    Commands.GyroDriveREG(speed=-120, distance=-37),
    Commands.GyroPivot(angle=12),
    Commands.ActMotorTime(speed=275, time=1600),
    Commands.ActMotorTime(speed=-200, time=1000),
    Commands.ActMotorTime(speed=275, time=1600),
    Commands.ActMotorTime(speed=-200, time=3000),
    Commands.GyroPivot(angle=-12),
    Commands.ShiftGear(gear=2, wait=False),
    Commands.LineSquare(approachSpeed=75, returnTime=4000),
    Commands.DriveMM(speed=-100, distance=-18),
    Commands.GyroPivot(angle=-91),
    # Drive to seabed
    Commands.GyroDrive(speed=-150, distance=90),
    Commands.ActMotorTime(speed=-200, time=2200),
    Commands.GyroDrive(speed=-150, distance=60),
    # Lift sample
    Commands.ActMotorTime(speed=200, time=2500),
    # Drive away
    Commands.GyroDrive(speed=-175, distance=90),
    Commands.ActMotorTime(speed=200, time=2500),
    Commands.GyroDrive(speed=-175, distance=200),
    Commands.GyroPivot(angle=-71),
    # Drive to home
    Commands.DriveMM(speed=250, distance=777),
  ]