import Commands

def m03():
  return [
    # Push out
    Commands.GyroDrive(speed=75, distance=100),
    # Return to home
    Commands.DriveMM(speed=75, distance=-100),
  ]