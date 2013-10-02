from __future__ import division

from state import * 
import commands, core, util, pose, percepts
import time
from math import pi, sqrt
from random import randint

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand1 = StandNode()
    goalie = GoalieNode()
    
    self._adt(start, N, TurnHeadNode(0.0, 1.0, False), S, TiltHeadNode(-26.5), C, stand1, C, goalie)
    
class GoalieNode(Node):
  MY_START = 0
  
  MY_WAIT_FOR_BALL_TO_MOVE = 1
  
  MY_BLOCK_BALL = 2
  
  def __init__(self):
    super(GoalieNode, self).__init__()
    self.myState = GoalieNode.MY_START
  
  def run(self):
    if self.myState == GoalieNode.MY_START:
      commands.stand()
      self.myState = GoalieNode.MY_WAIT_FOR_BALL_TO_MOVE
    
    if self.myState == GoalieNode.MY_WAIT_FOR_BALL_TO_MOVE:
      ball = core.world_objects.getObjPtr(core.WO_BALL)
      vSq = ball.velX ** 2 + ball.velY ** 2
      print vSq
      if vSq >= 2500.0:
        self.myState = GoalieNode.MY_BLOCK_BALL
    
    if self.myState == GoalieNode.MY_BLOCK_BALL:
      ball = core.world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        whiteLine = core.world_objects.getObjPtr(core.WO_OPP_GOAL_LINE)
        print whiteLine.fieldLineIndex
        if whiteLine.fieldLineIndex == 0:
          print "white line not seen"
          if ball.imageCenterX < 155:
            print "BALL LEFT"
            commands.setWalkVelocity(0.2, 1.0, 0.0)
          elif ball.imageCenterX > 165:
            print "BALL RIGHT"
            commands.setWalkVelocity(0.2, -1.0, 0.0)
          else:
            commands.stand()
        else:
          if whiteLine.fieldLineIndex >= 8:
            turnAngle = -pi / 12
            whiteLine.fieldLineIndex -= 8
          else:
            turnAngle = pi / 12
          if whiteLine.fieldLineIndex == 0:
            commands.setWalkVelocity(0.2, 0.0, turnAngle)
          if whiteLine.fieldLineIndex == 1:
            if ball.imageCenterX < 155:
              print "BALL LEFT"
              commands.setWalkVelocity(-0.2, 1.0, turnAngle)
            elif ball.imageCenterX > 165:
              print "BALL RIGHT"
              commands.setWalkVelocity(-0.2, -1.0, turnAngle)
            else:
              commands.setWalkVelocity(-0.2, 0.0, turnAngle)
          elif whiteLine.fieldLineIndex == 2:
            commands.setWalkVelocity(0.0, -1.0, turnAngle)
          elif whiteLine.fieldLineIndex == 3:
            commands.setWalkVelocity(-0.2, -1.0, turnAngle)
          elif whiteLine.fieldLineIndex == 4:
            commands.setWalkVelocity(0.0, 1.0, turnAngle)
          elif whiteLine.fieldLineIndex == 5:
            commands.setWalkVelocity(-0.2, 1.0, turnAngle)
          elif whiteLine.fieldLineIndex == 6:
            print "THIS SHOULD NOT HAPPEN"
            if randint(0, 1) == 0:
              commands.setWalkVelocity(-0.5, 0.0, turnAngle)
            else:
              commands.setWalkVelocity(0.5, 0.0, turnAngle)
          elif whiteLine.fieldLineIndex == 7:
            commands.setWalkVelocity(-0.5, 0.0, turnAngle)  
      else:
        commands.stand()

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
    if self.getTime() > 1.0:
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
    if self.getTime() > self.turnTime:
      self.postSuccess()



