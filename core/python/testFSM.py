from state import * 
import commands, core, util, pose, percepts
import time
from math import pi

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand1 = StandNode()
    stand2 = StandNode()
    walk = WalkNode(1, 0.1, 0, 0)
    locateBall = LocateBallNode(False)
    locateBlueWall = LocateBlueWallNode()
    tiltHead = TiltHeadNode(-21.1)
    walkLeft = WalkLeftNode(0.1)
    walkRight = WalkRightNode(0.1)

    self._adt(start, N, stand1)
    self._adt(stand1, C, WalkNode(10.0, 0.1, 0, 0), C, WalkNode(5.0, 0, 0, pi / 4), C, StandNode(), C, TurnHeadNode(-pi / 3, 2, True), C, TurnHeadNode(pi / 3, 2, True), C, stand2)
    self._adt(stand2, C, tiltHead)
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
    if ball.seen:
      core.speech.say("I see the ball")
    else:
      core.speech.say("Where is the ball")
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
  def __init__(self, walkTime, x, y, theta):
    super(WalkNode, self).__init__()
    self.walkTime = walkTime
    self.x = x
    self.y = y
    self.theta = theta
      
  def run(self):
    print percepts.joint_angles[core.LKneePitch]  # XXX: remove
    commands.setWalkVelocity(self.x, self.y, self.theta)
    if self.getTime() > self.walkTime:
      commands.stand()
      self.postSuccess()

class WalkLeftNode(Node):
  def __init__(self, walkTime):
    super(WalkLeftNode, self).__init__()
    self.walkTime = walkTime
  
  def run(self):
    commands.setWalkVelocity(0.1, 0, pi / 18)
    if self.getTime() > self.walkTime:
      commands.stand()
      self.postSuccess()

class WalkRightNode(Node):
  def __init__(self, walkTime):
    super(WalkRightNode, self).__init__()
    self.walkTime = walkTime
      
  def run(self):
    commands.setWalkVelocity(0.1, 0, -pi / 18)
    if self.getTime() > self.walkTime:
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

class TurnHeadNode(Node):
  def __init__(self, angle, turnTime, relative):
    super(TurnHeadNode, self).__init__()
    self.angle = angle
    self.turnTime = turnTime
    self.relative = relative
  
  def run(self):
    commands.setHeadPan(self.angle, self.turnTime, self.relative)
    if self.getTime() > self.turnTime + 0.1:
      self.postSuccess()



