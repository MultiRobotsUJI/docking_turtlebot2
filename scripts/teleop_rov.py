#!/usr/bin/env python

from mavros_msgs.msg import OverrideRCIn
import rospy
from pynput import keyboard
import os


class TeleopRov(object):
  def __init__(self) -> None:
    self.motor_publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    self.wait_time = 0.1
    self.base_channel = [1500, 1500, 1500, 1500, 1500, 1500, 1200, 1500, 1100, 1100, 0, 0, 0, 0, 0, 0, 0, 0]
    self.x_forward = 1600
    self.x_backward = 1400
    self.y_left = 1400
    self.y_right = 1600
    self.z_up = 1600
    self.z_down = 1400
    self.roll_left = 1400
    self.roll_right = 1600
    self.pitch_up = 1600
    self.pitch_down = 1400
    self.yaw_left = 1400
    self.yaw_right = 1600
    self.stop = 1500
  
    self.commands = {'w': {'var':'x', 'val':self.x_forward, 'name':'x_forward'},
                     's': {'var': 'x', 'val': self.x_backward, 'name': 'x_backward'},
                     'a': {'var': 'y', 'val': self.y_left, 'name': 'y_left'},
                     'd': {'var': 'y', 'val': self.y_right, 'name': 'y_right'},
                     'q': {'var': 'z', 'val': self.z_up, 'name': 'z_up'},
                     'e': {'var': 'z', 'val': self.z_down, 'name': 'z_down'},
                     'j': {'var': 'roll', 'val': self.roll_left, 'name': 'roll_left'},
                     'l': {'var': 'roll', 'val': self.roll_right, 'name': 'roll_right'},
                     'i': {'var': 'pitch', 'val': self.pitch_up, 'name': 'pitch_up'},
                     'k': {'var': 'pitch', 'val': self.pitch_down, 'name': 'pitch_down'},
                     'u': {'var': 'yaw', 'val': self.yaw_left, 'name': 'yaw_left'},
                     'o': {'var': 'yaw', 'val': self.yaw_right, 'name': 'yaw_right'},
                     ' ': {'var': 'stop', 'val': self.stop, 'name': 'stop'},
                     }
    x, y, z, roll, pitch, yaw = 1500, 1500, 1500, 1500, 1500, 1500
    self.command_mapping = {'x': x, 'y': y, 'z': z,
                            'roll': roll, 'pitch': pitch, 'yaw': yaw}
    rospy.Timer(rospy.Duration(0.001), self.timer_callback)
    rospy.Timer(rospy.Duration(0.001), self.on_press_timer)
    
  def map_key(self, key):
    available_keys = self.commands.keys()
    for k in available_keys:
      if key == keyboard.KeyCode.from_char(k):
        return k
    return None
  
  def on_press_timer(self, key):
      with keyboard.Events() as events:
          # Block for as much as possible
        event = events.get(self.wait_time)
        if event is not None:
            key = self.map_key(event.key)
            if key is not None:
              # os.system('clear')
              command = self.commands[key]
              print(command['name'])
              if command['var'] == 'stop':
                for i, key in enumerate(self.command_mapping.keys()):
                  self.command_mapping[key] = self.base_channel[i]
              else:
                self.command_mapping[command['var']] = command['val']
              return
      x, y, z, roll, pitch, yaw = 1500, 1500, 1500, 1500, 1500, 1500
      self.command_mapping = {'x': x, 'y': y, 'z': z, 'roll': roll, 'pitch': pitch, 'yaw': yaw}
  
  def timer_callback(self, event):
    msg = OverrideRCIn()
    msg.channels = [self.command_mapping['x'], self.command_mapping['y'], self.command_mapping['z'], self.command_mapping['roll'], self.command_mapping['pitch'], self.command_mapping['yaw'],
                              1200, 1500, 1100, 1100, 0, 0, 0, 0, 0, 0, 0, 0]
    # print(msg.channels)
    self.motor_publisher.publish(msg)


if __name__ == '__main__':
  rospy.init_node('teleop_rov')
  teleop_rov = TeleopRov()
  rospy.sleep(1)
  # keyboard.hook(my_keyboard_hook)
  rospy.spin()