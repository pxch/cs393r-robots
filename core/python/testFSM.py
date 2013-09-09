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
    locateBall = LocateBallNode()
    locateBlueWall = LocateBlueWallNode()
    tiltHead = TiltHeadNode(-35)

    self._adt(start, N, stand)
    self._adt(stand, C, tiltHead)
    self._adt(tiltHead, C, locateBlueWall)
    self._adt(locateBlueWall, S(BlueWallLocation.Far), walk, S, locateBlueWall)
    self._adt(locateBlueWall, S(BlueWallLocation.Near), sit)
    self._adt(sit, C, finish)

class BallLocation:
  Left = 0
  Right = 1
  Middle = 2
  
class BlueWallLocation:
  Far = 0
  Near = 1

class LocateBallNode(Node):
  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    print ball.imageCenterX, ball.imageCenterY
    if self.getTime() > 0.2:
      if ball.imageCenterX < 150:
        choice = 0
      elif ball.imageCenterX > 170:
        choice = 1
      else:
        choice = 2
      self.postSignal(choice)  

class LocateBlueWallNode(Node):
  def run(self):
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    print goal.imageCenterX, goal.imageCenterY, goal.radius
    if self.getTime() > 0.5:
      if goal.radius < 0.5:
        choice = 0
      else:
        choice = 1
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

class WalkNode(Node):
  def run(self):
    commands.setWalkVelocity(1, 0, 0)
    core.speech.say("I am walking toward the goal")
    if self.getTime() > 10.0:
      commands.stand()
      self.postSuccess()
      
class BallLeftNode(Node):
  def __init__(self):
    super(BallLeftNode, self).__init__()

  def run(self):
    commands.setHeadPan(pi / 18, 0.2, True)
    if self.getTime() > 0.3:
      # XXX: don't use time to terminate
      self.postSuccess()
 
class BallRightNode(Node):
  def __init__(self):
    super(BallRightNode, self).__init__()

  def run(self):
    commands.setHeadPan(-pi / 18, 0.2, True)
    if self.getTime() > 0.3:
      # XXX: don't use time to terminate
      self.postSuccess()

class TiltHeadNode(Node):
  def __init__(self, tilt):
    super(TiltHeadNode, self).__init__()
    self.tilt = tilt
  
  def run(self):
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 2.0:
      # XXX: don't use time to terminate
      self.postCompleted()
