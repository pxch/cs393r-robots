from __future__ import division

from state import * 
import commands, core, util, pose, percepts
import time
from math import pi, sqrt
from random import randint
from collections import deque

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand1 = StandNode()
    goalie = GoalieNode()
    
    self._adt(start, N, TiltHeadNode(-26.5), C, goalie)
    
class GoalieNode(Node):
  MY_START = 0
  
  MY_WAIT_FOR_BALL_TO_MOVE = 1
  
  MY_BLOCK_BALL = 2
  
  def __init__(self):
    super(GoalieNode, self).__init__()
    self.myState = GoalieNode.MY_START
    self.ballTrackQueue = deque()
  
  def run(self):
    if self.myState == GoalieNode.MY_START:
      commands.stand()
      self.myState = GoalieNode.MY_WAIT_FOR_BALL_TO_MOVE
    
    if self.myState == GoalieNode.MY_WAIT_FOR_BALL_TO_MOVE:
      ball = core.world_objects.getObjPtr(core.WO_BALL)
      if ball.ballBlobIndex != 0:
        self.myState = GoalieNode.MY_BLOCK_BALL
    
    if self.myState == GoalieNode.MY_BLOCK_BALL:
      core.speech.say("moving")

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



