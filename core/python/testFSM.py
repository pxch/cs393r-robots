from state import * 
import commands, core, util, pose, percepts
import time
from math import pi

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    stand = StandNode()
    sit = SitNode()
    searchBall = SearchBallNode()
    searchGoal = SearchGoalNode()
    kickBall = KickBallNode()

    self._adt(start, C, stand)
    # self._adt(stand, C, searchBall)
    # self._adt(searchBall, S, searchGoal)
    # self._adt(searchGoal, S, kickBall)
    # self._adt(kickBall, S, sit)
    # self._adt(sit, C, finish)

class SearchBallNode(Node):
  def __init__(self):
    super(SearchBallNode, self).__init__()

  def run(self):
    core.speech.say("searching the ball")

class SearchGoalNode(Node):
  def __init__(self):
    super(SearchGoalNode, self).__init__()

  def run(self):
    core.speech.say("searching the goal")

class KickBallNode(Node):
  def __init__(self):
    super(KickBallNode, self).__init__()

  def run(self):
    core.speech.say("kicking the ball")
    
class StandNode(Node):
  def __init__(self):
    super(StandNode, self).__init__()
    self.task = pose.Stand()

  def run(self):
    core.speech.say("stand up")
    self.task.processFrame()
    if self.task.finished():
      self.postCompleted()

class SitNode(Node):
  def __init__(self):
    super(SitNode, self).__init__()
    self.task = pose.Sit()

  def run(self):
    core.speech.say("sit down")
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
#    print percepts.joint_angles[core.LKneePitch]  # XXX: remove
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
    core.speech.say("turn head")
    commands.setHeadPan(self.angle, self.turnTime, self.relative)
    if self.getTime() > self.turnTime + 10.0:
      self.postSuccess()



