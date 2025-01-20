import Commands

def m02():
  return [
    # Start Driving
    Commands.GyroDrive(speed=175, distance=359),
    # Turn to M06
    Commands.GyroPivot(angle=68),
    # Arggresively ram with passive grabber
    Commands.GyroDrive(speed=200, distance=135),
    # Back outs
    Commands.GyroDrive(speed=175, distance=-135),
    # Turn To align with M01
    Commands.GyroPivot(angle=90),
    ]