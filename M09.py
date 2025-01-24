import Commands

def m09():
  return [
    Commands.GyroDrive(speed=-175, distance=660),
    Commands.GyroPivot(angle=30),
    Commands.GyroDrive(speed=-120, distance=100),
    Commands.GyroDrive(speed=-175, distance=-150),
    Commands.GyroPivot(angle=37),
    Commands.GyroDrive(speed=-150, distance=180),
  ]