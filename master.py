from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (ColorSensor, GyroSensor)
from pybricks.media.ev3dev import SoundFile, Image, Font
from pybricks.parameters import Button
from pybricks.tools import wait

from robot import Robot_Plus
#from Commands.py import ShiftGear


from M02 import m02
from M03 import m03
from M05 import m05
from M06 import m06
from M08 import m08
from M09 import m09
from M15 import m15


import threading
import _thread
import math

class Master_Main():
  def __init__(self):
    self.robot = Robot_Plus()
    self.ev3 = EV3Brick()
    self.missions = [
      ["M09", m09(), "IMAGES/missions/M09"],
      ["M05", m05(), "IMAGES/missions/M05"],
      ["M06", m06(), "IMAGES/missions/M06"], 
      ["M08", m08(), "IMAGES/missions/M08"],
      ["M02", m02(), "IMAGES/missions/M02"],
      ["M03", m03(), "IMAGES/missions/M03"],
      ["M15", m15(), "IMAGES/missions/M15"]
    ]
    self.mission_is_running = False
    self.count = 0
    self.mission_font = Font('lucidia console', size=32, monospace=True)
    self.init_font = Font('lucidia console', size=24, monospace=True)
    self.ev3.screen.set_font(self.init_font)

    self.button_height = 93
    self.init_images = ["IMAGES/init-1", "IMAGES/init-2", "IMAGES/init-3", "IMAGES/init-4", "IMAGES/init-5", "IMAGES/init-6"]

  def draw_button(self, pressed_button_image):
    self.ev3.screen.draw_image(0, self.button_height, pressed_button_image)
    wait(200)
    self.ev3.screen.draw_image(0, self.button_height, "IMAGES/buttons/buttons-empty")

  def display(self, run_num, button):
    current_mission = self.missions[run_num]
    self.ev3.screen.draw_image(0, 0, current_mission[2])
    self.draw_button(button)

  def play(self, run_number):
    run = self.missions[run_number]
    commands = run[1]
    for command in commands:
      if self.mission_is_running == False:
        self.ev3.speaker.beep(frequency=800)
        break
      command.run(self.robot)
    commands = []
    self.mission_is_running = False

  def play_mission(self, run_number):
    self.mission_is_running = True
    _thread.start_new_thread(self.play, (run_number))

    self.ev3.screen.draw_image(0, self.button_height, "IMAGES/buttons/buttons-abort")

    while self.mission_is_running:
      if  self.ev3.buttons.pressed() == [Button.DOWN]:
        self.mission_is_running = False
        self.ev3.speaker.beep(frequency=800)
        self.ev3.screen.draw_image(0, self.button_height, "IMAGES/buttons/buttons-abort-pressed")
    
    wait(500)

  def module(self):
    run_num = 0

    while True:
      while self.ev3.buttons.pressed() == []:
        wait(50)
      buttons = self.ev3.buttons.pressed()
      button = "IMAGES/buttons/buttons-empty"

      if buttons == [Button.CENTER]:
        # Draw Buttons
        self.draw_button("IMAGES/buttons/buttons-pressed-center")
        # Play current module
        self.robot.gyro_sensor.reset_angle(0)
        self.play_mission(tuple([run_num]))
        # Auto Advance
        run_num += 1
        if run_num >= len(self.missions):
          run_num = 0
      elif buttons == [Button.RIGHT]:
        # Draw Buttons
        button = "IMAGES/buttons/buttons-pressed-right"
        # Move to next module
        if run_num >= len(self.missions) - 1:
          run_num = 0
        else:
          run_num += 1
      elif buttons == [Button.LEFT]:
        # Draw Buttons
        button = "IMAGES/buttons/buttons-pressed-left"
        # Move to last module
        run_num -= 1
        if run_num <= -1:
          run_num = len(self.missions) - 1

      self.display(run_num, button)
    # put Gear var here?
  def calibrate_gyro(self):
    error = 0
    self.count = 0
    drift = False
    while self.ev3.buttons.pressed() != [Button.UP]:
      if error != 0:
        print("Gyro is drifing!")
        drift = True
        break

      if self.count >= 3:
        print("No drift here!")
        drift = False
        break
  
      error = (self.robot.gyro_sensor.angle())
      self.count += 1
      print("Error is", error,". . ", "Looped", self.count, "times.")
      wait(1000)

  def start(self):
    cal_g = threading.Thread(target=self.calibrate_gyro)
    cal_g.start()

    while self.count < 3:
      # Calibrate gyro & color sensors
      num = self.count
      self.ev3.screen.draw_image(0, 0, self.init_images[num])
      wait(900)

      # Check for skip
      if self.ev3.buttons.pressed() == [Button.UP]:
        break

    self.robot.calibrate_color()

    for x in range(0, 2):
      self.ev3.screen.draw_image(0, 0, "IMAGES/ready-1")
      self.ev3.speaker.beep(frequency=440)
      wait(200)
      self.ev3.screen.draw_image(0, 0, "IMAGES/ready-2")
      wait(200)
    self.ev3.screen.draw_image(0, 0, "IMAGES/ready-1")
    wait(200)
    
    self.ev3.screen.set_font(self.mission_font)
    self.display(0, "IMAGES/buttons/buttons-empty")
    self.module()