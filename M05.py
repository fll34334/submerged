import Commands

def m05():
  return [
    # Drive from angled jig
    Commands.GyroDrive(speed=-175, distance=868),
    # Turn around
    Commands.GyroPivot(angle=145),
    # Line up
    Commands.GyroDrive(speed=-175, distance=101),
    # Turn To LineSquare
    Commands.GyroPivot(angle=-90),
    # LineSquare
    Commands.LineSquare(approachSpeed=75, returnTime=4000),
    Commands.DriveMM(speed=-100, distance=-12),
    Commands.GyroPivot(angle=-90),
    # Drive to seabed
    Commands.GyroDrive(speed=-175, distance=440),
    Commands.GyroPivot(angle=-66),
    # Drive to home
    Commands.DriveMM(speed=250, distance=777)
  ]