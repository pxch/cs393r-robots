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
    locateBall = LocateBallNode()

    self._adt(start, N, stand)
    self._adt(stand, C, locateBall)
    self._adt(locateBall, S(BallLocation.Left), BallLeftNode(), S, locateBall)
    self._adt(locateBall, S(BallLocation.Right), BallRightNode(), S, locateBall)
    self._adt(locateBall, S(BallLocation.Middle), locateBall)

class BallLocation:
  Left = 0
  Right = 1
  Middle = 2

class LocateBallNode(Node):
  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)  
    if self.getTime() > 0.2:
      if ball.imageCenterX < 150:
        choice = 0
      elif ball.imageCenterX > 170:
        choice = 1
      else:
        choice = 2
      self.postSignal(choice)  

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
    commands.setWalkVelocity(1, 0, pi / 6)
    core.speech.say("I am walking")
    if self.getTime() > 10.0:
      commands.stand()
      self.postSuccess()


class TrackBallNode(Node):
  def __init__(self):
    super(TrackBallNode, self).__init__()

  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    print ball.imageCenterX, ball.imageCenterY
    if ball.imageCenterX < 160:
      commands.setHeadPan(pi / 18, 0.1)
    else:
      commands.setHeadPan(-pi / 18, 0.1)

    if self.getTime() > 30.0:
      self.postSuccess()
      
class BallLeftNode(Node):
  def __init__(self):
    super(BallLeftNode, self).__init__()

  def run(self):
    commands.setHeadPan(pi / 18, 0.2)
    if self.getTime() > 0.2:
      self.postSuccess()
 
class BallRightNode(Node):
  def __init__(self):
    super(BallRightNode, self).__init__()

  def run(self):
    commands.setHeadPan(-pi / 18, 0.2)
    if self.getTime() > 0.2:
      self.postSuccess()
 

