#!/usr/bin/env python

import core

joint_angles = [0] * core.NUM_JOINTS
sensors = [0] * core.NUM_SENSORS

def update():
  global joint_angles
  for i in range(core.NUM_JOINTS):
    joint_angles[i] = core.pythonC.getFloat(core.joint_angles.values_, i)

  for i in range(core.NUM_SENSORS):
    sensors[i] = core.pythonC.getFloat(core.sensors.values_, i)
