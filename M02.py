import Commands

def m02():
  return [
    Commands.GyroDrive(speed=175, distance=220),
    #Commands.ActMotorTime(motor=1, speed=250, time=3000),
    #Commands.GyroDrive(speed=175, distance=8),
    Commands.GyroPivot(angle=25),
    #Commands.ActMotorTime(motor=1, speed=250, time=4000),
    #Commands.ActMotorTime(motor=1, speed=-250, time=7000),
    #add hook code
    #Commands.GyroPivot(angle=135),
    Commands.GyroDrive(speed=175, distance=145),
    Commands.GyroPivot(angle=65),
    Commands.GyroDrive(speed=175, distance=135),
    ]