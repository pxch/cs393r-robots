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
    locateBall = LocateBallNode(False)
    locateBlueWall = LocateBlueWallNode()
    tiltHead = TiltHeadNode(-21.1)
    walkLeft = WalkLeftNode()
    walkRight = WalkRightNode()

    self._adt(start, N, stand)
    self._adt(stand, C, tiltHead)
    self._adt(tiltHead, C, locateBall)
    self._adt(locateBall, S(BallLocation.Left), walkLeft, S, locateBall)
    self._adt(locateBall, S(BallLocation.Right), walkRight, S, locateBall)
    self._adt(locateBall, S(BallLocation.Middle), walk, S, locateBall)

class BallLocation:
  Left = 0
  Right = 1
  Middle = 2

class LocateBallNode(Node):
  def __init__(self, fromTop):
    super(LocateBallNode, self).__init__()
    self.fromTop = fromTop
  
  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    choice = BallLocation.Middle
    if ball.fromTopCamera == self.fromTop:
      print ball.imageCenterX, ball.imageCenterY
      if ball.imageCenterX < 150:
        choice = BallLocation.Left
      elif ball.imageCenterX > 170:
        choice = BallLocation.Right
      else:
        choice = BallLocation.Middle
    self.postSignal(choice)  

class BlueWallLocation:
  FarRight = 0
  FarLeft = 1
  Near = 2

class LocateBlueWallNode(Node):
  def run(self):
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    print goal.imageCenterX, goal.imageCenterY, goal.radius
    if self.getTime() > 0.5:
      choice = BlueWallLocation.Near
      if goal.fromTopCamera:
        if goal.radius < 0.5:
          if goal.imageCenterX < 160:
            choice = BlueWallLocation.FarLeft
          else:
            choice = BlueWallLocation.FarRight
        else:
          choice = BlueWallLocation.Near
      self.postSignal(choice)

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
    commands.setWalkVelocity(0.1, 0, 0)
    if self.getTime() > 1.0:
      commands.stand()
      self.postSuccess()

class WalkLeftNode(Node):
  def run(self):
    commands.setWalkVelocity(0.1, 0, pi / 18)
    if self.getTime() > 1.0:
      commands.stand()
      self.postSuccess()

class WalkRightNode(Node):
  def run(self):
    commands.setWalkVelocity(0.1, 0, -pi / 18)
    if self.getTime() > 1.0:
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
