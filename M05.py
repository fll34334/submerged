import Commands

def m05():
  return [
    # Drive from angled jig
    Commands.GyroDrive(speed=-175, distance=898),
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
    Commands.GyroDriveREG(speed=-120, distance=-44),
    Commands.GyroPivot(angle=12),
    Commands.ActMotorTime(speed=200, time=1600),
    Commands.ActMotorTime(speed=-200, time=2200),
    Commands.GyroPivot(angle=-12),
    Commands.GyroDriveREG(speed=-120, distance=15),
    Commands.GyroPivot(angle=-90),
    Commands.ShiftGear(gear=2),
    # Drive to seabed
    Commands.GyroDrive(speed=-150, distance=90),
    Commands.ActMotorTime(speed=-200, time=2000),
    Commands.GyroDrive(speed=-150, distance=57),
    # Lift sample
    Commands.ActMotorTime(speed=200, time=2500),
    # Drive away
    Commands.GyroDrive(speed=-175, distance=290),
    Commands.GyroPivot(angle=-71),
    # Drive to home
    Commands.DriveMM(speed=250, distance=777),
  ]