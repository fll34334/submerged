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
    Commands.GyroPivot(angle=90, speed=75),
    # drive to M01
    # Commands.GyroDrive (speed=175, distance=-145),
    # Commands.ActMotorTime(motor=4, speed=-700, time=3000),
    # Commands.ActMotorTime(motor=4, speed=700, time=3000),
    # Commands.GyroDrive(speed=175, distance=-120),
    # Commands.GyroPivot(angle=32),
    # Commands.ActMotorTime(motor=4, speed=-700, time=3000),
    # Commands.ActMotorTime(motor=4, speed=700, time=3000, wait=False),
    # Commands.ActMotorTime(motor=1, speed=300, time=1000),
    # Commands.ActMotorTime(motor=1, speed=-300, time=1000, wait=False),
    # Commands.GyroPivot(angle=-34),
    # Commands.DriveMM(speed=250, distance=-850),
    ]