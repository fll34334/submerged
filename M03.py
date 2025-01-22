import Commands

def m03():
  return [
    # Push out
    Commands.GyroDriveREG(speed=60, distance=-100),
    # Return to home
    Commands.GyroDriveREG(speed=60, distance=100),
  ]