from state import * 
import commands, core, util, pose
import time
from math import pi

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand = StandNode()
    walk = WalkNode()
    turnHead = TurnHeadNode()
    
    self._adt(start, N, stand)
    self._adt(stand, C, walk)
    self._adt(walk, S, turnHead)
    self._adt(turnHead, S, sit)
    self._adt(sit, C, finish)

class FoundBall(object):
  Yes = 1
  No = 0
    
class FindBallNode(Node):
  def __init__(self):
    super(FindBallNode, self).__init__()
  
  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)    
    if ball.seen:
      self.postSignal(FindBall.Yes)
    else:
      self.postSignal(FindBall.No)

class SpeakNode(Node):
  def __init__(self, phrase):
    super(SpeakNode, self).__init__()
    self.phrase = phrase

  def run(self):
    if self.getFrames() == 0:
      core.speech.say(self.phrase)
    if self.getTime() > 4.0:
      self.postSuccess()

class SitNode(Node):
  def __init__(self):
    super(SitNode, self).__init__()
    self.task = pose.Sit()

  def run(self):
    self.task.processFrame() 
    if self.task.finished():
      self.postCompleted()

class StandNode(Node):
  def __init__(self):
    super(StandNode, self).__init__()
    self.task = pose.Stand()

  def run(self):
    self.task.processFrame()
    if self.task.finished():
      self.postCompleted()
      
class TurnHeadNode(Node):
  # XXX: complete this
  
  def __init__(self):
    super(TurnHeadNode, self).__init__()
    
  def run(self):
    core.speech.say("I am turning head")
    commands.setHeadPan(pi / 3, 2)
    if self.getTime() > 10.0:
      self.postSuccess()

class WalkNode(Node):
  def run(self):
    commands.setWalkVelocity(.5, .5, 0)
    core.speech.say("I am walking")
    if self.getTime() > 10.0:
      commands.stand()
      self.postSuccess()





