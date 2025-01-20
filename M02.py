import Commands

def m02():
  return [
    # Start Driving
    Commands.GyroDrive(speed=175, distance=220),
    # Pivot to avoid m01
    Commands.GyroPivot(angle=25),
    # Drive to align with M06
    Commands.GyroDrive(speed=175, distance=145),
    # Turn to M06
    Commands.GyroPivot(angle=65),
    # Ram with passive grabber
    Commands.GyroDrive(speed=175, distance=135),
    # Back out
    Commands.GyroDrive(speed=175, distance=-175),
    # Turn To align with M01
    Commands.GyroPivot(angle=-90),
    ]