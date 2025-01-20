import Commands

def m02():
  return [
    # Start Driving
    Commands.GyroDrive(speed=175, distance=367),
    # Turn to M06
    Commands.GyroPivot(angle=65),
    # Arggresively ram with passive grabber
    Commands.GyroDriveREG(speed=250, distance=185),
    # Back outs
    Commands.GyroDrive(speed=175, distance=-65),
    # Turn To align with M01
    Commands.GyroPivot(angle=90),
    #drive to M01
    Commands.GyroDrive (speed=175, distance=-130),
    Commands.ActMotorTime(motor=4, speed=-400, time=1800),
    Commands.ActMotorTime(motor=4, speed=400, time=1800),
    ]