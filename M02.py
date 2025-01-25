import Commands

def m02():
  return [
    # Start Driving
    Commands.GyroDrive(speed=175, distance=390),
    # Turn to M06
    Commands.GyroPivot(angle=68, speed=75),
    # Arggresively ram with passive grabber
    Commands.DriveMM(speed=175, distance=-410),
    # Back outs while losing all sanity that you somehow still have left
    Commands.GyroDrive(speed=175, distance=-132),
    # Turn To align with M01
    Commands.GyroPivot(angle=87, speed=75),
    # Drive to M01
    Commands.GyroDrive(speed=175, distance=-160),
    # Do M01
    Commands.ActMotorTime(motor=4, speed=-350, time=6250),
    Commands.ActMotorTime(speed=350, time=2500),
    # Wall square
    Commands.DriveMM(speed=175, distance=200),
    Commands.DriveMM(speed=100, distance=50),
    # Align with M02
    Commands.GyroDriveREG(speed=125, distance=25),
    # Do M02
    Commands.ActMotorTime(speed=-350, time=5555),
    Commands.ActMotorTime(speed=350, time=4000),
    Commands.ActMotorTime(speed=350, time=5000, wait=False),
    # Back really outs
    Commands.GyroDrive(speed=-175, distance=-190, angle=-5),
    # Pivot to align with M03
    Commands.GyroPivot(speed=80, angle=16),
    # Drive to M03
    Commands.GyroDrive(speed=-120, distance=25),
    # Do M03
    Commands.ActMotorTime(motor=1, speed=-250, time=2400),
    Commands.ActMotorTime(motor=1, speed=200, time=2400),
    # Go to tacos-4-life and home to regain mental sanity (not really you cant ever get it back)
    Commands.DriveMM(speed=-175, distance=-800),
    ]