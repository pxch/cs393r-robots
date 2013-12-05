#!/usr/bin/env python

import core
from task import Task, HeadBodyTask, MachineTask
import pose, head, kicks
import commands, cfgstiff
import testFSM

def areDistinct(state1, state2):
  if state1 == core.INITIAL and state2 == core.FINISHED: return False
  if state1 == core.FINISHED and state2 == core.INITIAL: return False
  if state1 == state2: return False
  return True

def createStateTask(state):
  if state == core.INITIAL: return Initial()
  if state == core.READY: return Ready()
  if state == core.PLAYING: return Playing()
  if state == core.FINISHED: return Finished()
  if state == core.TESTING: return Testing()
  if state == core.PENALISED: return Penalised()
  return None

Penalised = Initial = Finished = pose.Sit

class Ready(HeadBodyTask):
  def __init__(self):
    HeadBodyTask.__init__(self, 
      head.Scan(period = 3.0, maxPan = 105.0 * core.DEG_T_RAD, numSweeps = 4),
      pose.Stand()
    )

  def run(self):
    commands.setStiffness()
    HeadBodyTask.run(self)

class Playing(MachineTask):
  def __init__(self):
    super(Playing, self).__init__(testFSM.TestMachine())

class Testing(Task):
  def __init__(self):
    super(Testing, self).__init__()
    self.task = kicks.Kick()

  def reset(self):
    super(KickNode, self).reset()
    self.task = kicks.Kick()
  
  def run(self):
    self.task.processFrame()
    self.task.finished()
    #self.finish()
