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
    # Argressivly ram 13 while banging head against wall in frustraition
    Commands.DriveMM(speed=175, distance=125),
    Commands.DriveMM(speed=150, distance=-185),
    # Back up to linesqare crying 
    Commands.LineSquare(approachSpeed=75, returnTime=4000),
    # Turn away and drop arm
    Commands.DriveMM(speed=-100, distance=-12),
    Commands.GyroPivot(angle=-90),
    # Drive to seabed
    Commands.GyroDrive(speed=-175, distance=430),
    Commands.GyroPivot(angle=-66),
    # Drive to home
    Commands.DriveMM(speed=250, distance=777, angle=1),
  ]