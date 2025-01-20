import Commands

def m02():
  return [
    # Start Driving
    Commands.GyroDrive(speed=175, distance=358),
    # Turn to M06
    Commands.GyroPivot(angle=71),
    # Arggresively ram with passive grabber
    Commands.GyroDriveREG(speed=175, distance=120),
    Commands.GyroDriveREG(speed=250, distance=75),
    # Back outs
    Commands.GyroDrive(speed=175, distance=-90),
    # Turn To align with M01
    Commands.GyroPivot(angle=90),
    #drive to M01
    Commands.GyroDrive (speed=175, distance=-145),
    Commands.ActMotorTime(motor=4, speed=-700, time=3000),
    Commands.ActMotorTime(motor=4, speed=700, time=3000),
    Commands.GyroDrive(speed=175, distance=-120),
    Commands.GyroPivot(angle=32),
    Commands.ActMotorTime(motor=4, speed=-700, time=3000),
    Commands.ActMotorTime(motor=4, speed=700, time=3000),
    Commands.ActMotorTime(motor=1, speed=300, time=1000),
    Commands.ActMotorTime(motor=1, speed=-300, time=1000),
    ]