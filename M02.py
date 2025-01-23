import Commands

def m02():
  return [
    # Start Driving
    Commands.GyroDrive(speed=175, distance=380),
    # Turn to M06
    Commands.GyroPivot(angle=68, speed=75),
    # Arggresively ram with passive grabber
    Commands.DriveMM(speed=175, distance=-380),
    # Back outs
    Commands.GyroDrive(speed=175, distance=-111),
    # Turn To align with M01
    Commands.GyroPivot(angle=89, speed=75),
    # drive to M01
    Commands.GyroDrive (speed=175, distance=-90),
    Commands.ActMotorTime (motor='4', speed=-250, time=3000),
    Commands.DriveMM (speed=175, distance=380),
    ]