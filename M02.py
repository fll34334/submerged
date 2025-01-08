import Commands

def m02():
  return [
    Commands.GyroDrive(speed=250, distance=400),
    Commands.GyroPivot(angle=47),
    Commands.GyroDrive(speed=250, distance=125),
    #raise captain hook
    #lower captain hook
    #back up
    #raise captain hook
    #lower captain hook
    #easton
    ]