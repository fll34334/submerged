import Commands

def mo8():
  return [
    Commands.ActMotorTime(motor='right', speed=500, time=1800, wait=False),
    Commands.GyroDrive(speed=200, distance=610),
    Commands.GyroDrive(speed=150, distance=-5),
    Commands.ActMotorTime(motor='right', speed=-600, time=1000),
    Commands.Pivot(angle=4),
    Commands.GyroDrive(speed=150, distance=90),
    Commands.ActMotorTime(motor='right', speed=400, time=4000, wait=False),
    Commands.DriveMM(speed=20, distance=-70),
    Commands.Wait(time=400),
    Commands.DriveMM(speed=20, distance=-35),
    Commands.ActMotorTime(motor='right', speed=-400, time=3000, wait=False),
    Commands.DriveMM(speed=250, distance=-700, angle=2),
  ]