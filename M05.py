import Commands

def m05():
  return [
    # Drive from angled jig
    Commands.GyroDrive(speed=-175, distance=898),
    # Turn around
    Commands.GyroPivot(angle=144),
    # Line up
    Commands.GyroDrive(speed=-175, distance=101),
    # Turn To LineSquare
    Commands.GyroPivot(angle=-90),
    # LineSquare
    Commands.LineSquare(approachSpeed=75, returnTime=4000),
    # M13 bump
    # Turn away and drop arm
    # NEED TO BACKUP TO GET SAMPLE!!!! otherwise good
    #Commands.GyroDrive(speed=-150, distance=-20),
    Commands.ActMotorTime(speed=200, time=1600),
    Commands.ActMotorTime(speed=-200, time=2200),
    Commands.GyroPivot(speed=200, angle=-90),
    Commands.ShiftGear(gear=2),
    # Drive to seabed
    Commands.GyroDrive(speed=-150, distance=90),
    Commands.ActMotorTime(speed=-200, time=1900),
    Commands.GyroDrive(speed=-150, distance=51),
    # Lift sample
    Commands.ActMotorTime(speed=200, time=2500),
    # Drive away
    Commands.GyroDrive(speed=-175, distance=290),
    Commands.GyroPivot(angle=-71),
    #Commands.GyroDrive(speed=-175, distance=90),
    # One wheel turn to home
    #Commands.DriveMotor(motor="left", angle=130, speed=170),
    # Drive to home
    Commands.DriveMM(speed=250, distance=777),
  ]