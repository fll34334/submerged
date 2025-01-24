import Commands

def m02():
  return [
    # Start Driving
    Commands.GyroDrive(speed=175, distance=385),
    # Turn to M06
    Commands.GyroPivot(angle=68, speed=75),
    # Arggresively ram with passive grabber
    Commands.DriveMM(speed=175, distance=-380),
    # Back outs
    Commands.GyroDrive(speed=175, distance=-108),
    # Turn To align with M01
    Commands.GyroPivot(angle=87, speed=75),
    # drive to M01
    Commands.GyroDrive(speed=175, distance=-165),
    Commands.ActMotorTime(motor=4, speed=-350, time=8000),
    Commands.ActMotorTime(motor=4, speed=350, time=1900),
    Commands.DriveMM(speed=175, distance=300),
    Commands.ActMotorTime(motor=4, speed=-350, time=7000),
    Commands.ActMotorTime(motor=4, speed=350, time=2500),
    Commands.ActMotorTime(speed=350, time=5500, wait=False),
    Commands.GyroDrive(speed=-175, distance=-290, angle=-5),
    Commands.GyroPivot(angle=18),
    Commands.GyroDrive(speed=-175, distance=180),
    Commands.ActMotorTime(speed=250, time=2400),
    Commands.ActMotorTime(speed=-200, time=2400),
    Commands.GyroDrive(speed=-175, distance=800, angle=20),

    # M03 backup 3rd run (same jig as M02, useing tiger)
    # Commands.GyroDrive(speed=-175, distance=385),
    # Commands.GyroPivot(angle=22, speed=75),
    # Commands.GyroDrive(speed=-175, distance=380),
    # Commands.ActMotorTime(speed=250, time=2400),
    # Commands.ActMotorTime(speed=-200, time=2400),
    # Commands.GyroDrive(speed=-175, distance=800, angle=20),
    ]