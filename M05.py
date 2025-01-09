import Commands

def m05():
  return [
<<<<<<< HEAD
    Commands.GyroDrive(speed=-175, distance=90),
    Commands.GyroPivot(angle=-57),
    Commands.GyroDriveREG(speed=-150, distance=750),
=======
    Commands.GyroDrive(speed=-175, distance=50),
    Commands.GyroPivot(angle=-55),
    Commands.GyroDrive(speed=-175, distance=490),
    Commands.GyroDrive(speed=-175, distance=350, angle=3),
>>>>>>> d201e84 (m05 better (not))
  ]