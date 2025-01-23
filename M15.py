import Commands

def m15():
  return [
    # Out to dump
    Commands.GyroDriveREG(speed=150, distance=200),
    # Return to home
    Commands.GyroDriveREG(speed=-150, distance=200),
  ]