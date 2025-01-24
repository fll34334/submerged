import Commands

def m15():
  return [
    # Out to dump
    Commands.GyroDriveREG(speed=70, distance=210), # want to do Commands.DriveMM(speed=150, distance=-200),
    # Return to home
    Commands.GyroDriveREG(speed=-70, distance=210),
  ]