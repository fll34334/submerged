import Commands

def m08():
  return [
    Commands.ActMotorTime(motor=1, speed=200, time=2000, wait=False),
    Commands.GyroDrive(speed=-175, distance=50),
    Commands.GyroPivot(angle=69),
    Commands.GyroDrive(speed=-175, distance=610),
    Commands.DriveMM(speed=100, distance=5),
    Commands.ActMotorTime(motor=1, speed=-250, time=1800),
    Commands.GyroDrive(speed=-120, distance=90),
    Commands.ActMotorTime(speed=250, time=5000, wait=False),
    Commands.DriveMM(speed=20, distance=-70),
    Commands.Wait(time=400),
    Commands.DriveMM(speed=20, distance=35),
    Commands.ActMotorTime(speed=-250, time=2000, wait=False),
    Commands.GyroPivot(angle=-40),
    Commands.GyroDrive(speed=-175, distance=180),
    Commands.GyroPivot(angle=27),
    Commands.GyroDrive(speed=270, distance=777),
  ]