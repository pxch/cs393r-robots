from state import * 
import commands, core, util, pose, percepts
import time
from math import pi
from random import randint

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand1 = StandNode()
    testWhiteLine = RandWalkGoalLine()

    self._adt(start, N, TiltHeadNode(-26.5), C, stand1, C, testWhiteLine)

class RandWalkGoalLine(Node):
  MY_START = 0
  
  MY_WALK = 1
  
  MY_SWITCH = 2
  
  WALK_FRAMES = 100
  
  def __init__(self):
    super(RandWalkGoalLine, self).__init__()   
    self.myState = RandWalkGoalLine.MY_START
    self.walkCount = 0
    
    # walk parameters 
    self.x = 0.0
    self.y = 0.0
  
  def run(self):
    if self.myState == RandWalkGoalLine.MY_START:
      commands.stand()
      self.myState = RandWalkGoalLine.MY_SWITCH
      
    elif self.myState == RandWalkGoalLine.MY_SWITCH:
      randDir = randint(1, 4)
      if randDir == 1:
        self.x = 0.5
        self.y = 0.0
      elif randDir == 2:
        self.x = -0.5
        self.y = 0.0
      elif randDir == 3:
        self.x = 0.0
        self.y = 0.5
      elif randDir == 4:
        self.x = 0.0
        self.y = -0.5
      self.myState = RandWalkGoalLine.MY_WALK
      
    elif self.myState == RandWalkGoalLine.MY_WALK:
      whiteLine = core.world_objects.getObjPtr(core.WO_OPP_GOAL_LINE)
      if whiteLine.fieldLineIndex == 0:
        commands.setWalkVelocity(self.x, self.y, 0.0)
      else:
        commands.stand()
      self.walkCount += 1
      if self.walkCount == RandWalkGoalLine.WALK_FRAMES:
        self.walkCount = 0
        self.myState = RandWalkGoalLine.MY_SWITCH

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
    core.speech.say("sit")
    self.task.processFrame() 
    if self.task.finished():
      self.postCompleted()

class StandNode(Node):
  def __init__(self):
    super(StandNode, self).__init__()
    self.task = pose.Stand()

  def run(self):
    core.speech.say("stand")
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

class TiltHeadNode(Node):
  def __init__(self, tilt):
    super(TiltHeadNode, self).__init__()
    self.tilt = tilt
  
  def run(self):
    commands.setStiffness()
    commands.setHeadTilt(self.tilt)
    if self.getTime() > 3.0:
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



