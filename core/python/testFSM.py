from state import * 
import commands, core, util, pose
import time

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand = StandNode()
    turnHead = TurnHeadNode()
    
    self._adt(start, N, stand)
    self._adt(stand, C, turnHead)
    self._adt(turnHead, C, finish)

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
  def __init__(self):
    super(TurnHeadNode, self).__init__()

  def run(self):
    commands.setStiffness()
    commands.setWalkVelocity(.5, 0, 0)
    if self.getTime() < 10:
      core.speech.say("you are an idoit you can't control me")
      commands.stand()
      self.postCompleted()







