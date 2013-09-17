from state import * 
import commands, core, util, pose, percepts
import time
from math import pi, fabs, copysign
from re import search

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    stand = StandNode()
    sit = SitNode()
    searchBall = SearchBallNode()
    searchGoal = SearchGoalNode()
    kickBall = KickBallNode()

    self._adt(start, N, stand)
    self._adt(stand, C, searchBall)
    self._adt(searchBall, S, searchGoal)
    self._adt(searchGoal, S, kickBall)
    self._adt(kickBall, S, sit)
    self._adt(sit, C, finish)

class SearchBallNode(Node):
  MY_SUCCESS = 0
  
  MY_START = 1
  
  MY_NO_BALL = 2
  
  MY_BALL_TOP_LEFT = 3
  MY_BALL_TOP_RIGHT = 4
  
  MY_BALL_BOTTOM_LEFT_FAR = 5
  MY_BALL_BOTTOM_LEFT_MID = 6
  MY_BALL_BOTTOM_LEFT_NEAR = 7
  # FAR NEAR boundary y value is 80
  MY_BALL_BOTTOM_RIGHT_FAR = 8
  MY_BALL_BOTTOM_RIGHT_MID = 9
  MY_BALL_BOTTOM_RIGHT_NEAR = 10
  
  def __init__(self):
    super(SearchBallNode, self).__init__()
    self.my_state = SearchBallNode.MY_START 
    
    # error = expected_location - current_location
    self.xErrInt = 0.0  # integral of x error
    self.yErrInt = 0.0  # integral of y error
    
    self.prevXErr = None
    self.prevYErr = None
  
  def PID(self, xErr, yErr):
    dErrDt = self.dErrDt(xErr, yErr)
    
    xInput = xErr + self.xErrInt - dErrDt[0]
    
    yInput = yErr + self.yErrInt - dErrDt[1]
    
    return (self.inputToMotor(xInput), self.inputToMotor(yInput))
  
  def inputToMotor(self, inValue):
    motorSpeed = 0
    if fabs(inValue) > 100.0:
      return copysign(inValue, motorSpeed)
    motorSpeed = inValue / 100.0
    return motorSpeed
  
  def dErrDt(self, xErr, yErr):
    """
    dErr / dt
    """
    if self.prevXErr == None:
      return (0.0, 0.0)
    return (xErr - self.prevXErr, yErr - self.prevYErr)
  
  def switchWalkState(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    
    if not ball.seen:
      self.my_state = SearchBallNode.my_state = SearchBallNode.MY_NO_BALL
      return
    
    if ball.fromTopCamera:
          
      if ball.imageCenterX < 160:
        self.my_state = SearchBallNode.MY_BALL_TOP_LEFT
        
      else:
        self.my_state = SearchBallNode.MY_BALL_TOP_RIGHT
        
    else:  # from bottom camera 
      
      if ball.imageCenterY < 70:  # far
        if ball.imageCenterX < 160:
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_LEFT_FAR
        else: 
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_RIGHT_FAR
        
      elif ball.imageCenterY > 90:  # near
        if ball.imageCenterX < 160:
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_LEFT_NEAR
        else:
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_RIGHT_NEAR
      
      else:  # ball is in bottom middle
        
        if ball.imageCenterX < 150:  # mid left
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_LEFT_MID
          
        elif ball.imageCenterX > 170:  # mid right
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_RIGHT_MID
      
        else:
          self.my_state = SearchBallNode.MY_SUCCESS

  def run(self):
    core.speech.say("searching the ball")
    
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    
    xErr = 0.0
    yErr = 0.0
    
    if self.my_state == SearchBallNode.MY_SUCCESS:
      self.postSuccess()
      
    elif self.my_state == SearchBallNode.MY_START:
      self.my_state = SearchBallNode.MY_NO_BALL
      
    elif self.my_state == SearchBallNode.MY_NO_BALL:
      commands.setWalkVelocity(0, 0, pi)
      
      if ball.seen:
        commands.stand()
        self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_TOP_LEFT:
      xErr = 160 - ball.imageCenterX
      yErr = 320 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[0], controlSignal[1], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_TOP_RIGHT:
      xErr = 160 - ball.imageCenterX
      yErr = 320 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[0], controlSignal[1], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_LEFT_FAR:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[0], controlSignal[1], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_LEFT_MID:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[0], 0.0, 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_LEFT_NEAR:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[0], controlSignal[1], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_RIGHT_FAR:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[0], controlSignal[1], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_RIGHT_MID:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[0], 0.0, 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_RIGHT_NEAR:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[0], controlSignal[1], 0.0)
      
      self.switchWalkState()
    
    self.xErrInt += xErr
    self.yErrInt += yErr
    
    self.prevXErr = xErr
    self.prevYErr = yErr
    
    print "ball seen: ", ball.seen, "xErr: ", xErr, "yErr", yErr

class SearchGoalNode(Node):
  def __init__(self):
    super(SearchGoalNode, self).__init__()

  def run(self):
    core.speech.say("searching the goal")
    
    self.postSuccess()

class KickBallNode(Node):
  def __init__(self):
    super(KickBallNode, self).__init__()

  def run(self):
    core.speech.say("kicking the ball")
    
    self.postSuccess()
    
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



