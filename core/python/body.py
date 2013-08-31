#!/usr/bin/env python

from task import Task
import util
import core

class ToPose(Task):
  def __init__(self, pose, time = 2.0):
    Task.__init__(self)
    self.pose = pose
    self.time = time

  def run(self):
    for i in range(2, core.NUM_JOINTS):
      val = core.DEG_T_RAD * util.getPoseJoint(i, self.pose)
      core.joint_commands.setJointCommand(i, val)

    core.joint_commands.send_body_angles_ = True
    core.joint_commands.body_angle_time = self.time * 1000.0
    core.walk_request.noWalk()
    core.kick_request.setNoKick()

    if self.getTime() > self.time:
      self.finish()
