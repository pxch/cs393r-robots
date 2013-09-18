from state import * 
import commands, core, util, pose, percepts, kicks
import time
from math import pi, fabs, copysign, atan
from re import search

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    stand = StandNode()
    sit = SitNode()
    searchBall = SearchBallNode()
    searchGoal = SearchGoalNode()
    readyToKick = KickBallNode()
    kick = KickNode()

    self._adt(start, N, TiltHeadNode(-26.5), C, stand)
    self._adt(stand, C, searchBall)
    self._adt(searchBall, S, searchGoal)
    self._adt(searchGoal, S, readyToKick)
    self._adt(readyToKick , S, kick)
    self._adt(kick, C, sit)
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
    K_I = 0.0005
    K_D = 0.0

    dErrDt = self.dErrDt(xErr, yErr)
    
    xInput = xErr + K_I * self.xErrInt - K_D * dErrDt[0]
    
    yInput = yErr + K_I * self.yErrInt - K_D * dErrDt[1]
    
    return (self.inputToMotor(xInput), self.inputToMotor(yInput))
  
  def inputToMotor(self, inValue):
    BOUNDARY_VAL = 400.0
    motorSpeed = 0
    if fabs(inValue) > BOUNDARY_VAL:
      copysign(inValue, motorSpeed)
    else:
      motorSpeed = inValue / BOUNDARY_VAL
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
      
      if ball.imageCenterY < 65:  # far
        if ball.imageCenterX < 160:
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_LEFT_FAR
        else: 
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_RIGHT_FAR
        
      elif ball.imageCenterY > 95:  # near
        if ball.imageCenterX < 160:
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_LEFT_NEAR
        else:
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_RIGHT_NEAR
      
      else:  # ball is in bottom middle
        
        if ball.imageCenterX < 145:  # mid left
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_LEFT_MID
          
        elif ball.imageCenterX > 175:  # mid right
          self.my_state = SearchBallNode.MY_BALL_BOTTOM_RIGHT_MID
      
        else:
          self.my_state = SearchBallNode.MY_SUCCESS

  def run(self):
    core.speech.say("searching the ball")
    
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    
    xErr = 0.0
    yErr = 0.0
    
    controlSignal = (0.0, 0.0)
    
    if self.my_state == SearchBallNode.MY_SUCCESS:
      commands.stand()
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
      commands.setWalkVelocity(controlSignal[1], controlSignal[0], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_TOP_RIGHT:
      xErr = 160 - ball.imageCenterX
      yErr = 320 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[1], controlSignal[0], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_LEFT_FAR:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[1], controlSignal[0], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_LEFT_MID:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[1], 0.0, 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_LEFT_NEAR:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[1], controlSignal[0], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_RIGHT_FAR:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[1], controlSignal[0], 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_RIGHT_MID:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[1], 0.0, 0.0)
      
      self.switchWalkState()
    
    elif self.my_state == SearchBallNode.MY_BALL_BOTTOM_RIGHT_NEAR:
      xErr = 160 - ball.imageCenterX
      yErr = 80 - ball.imageCenterY
      
      controlSignal = self.PID(xErr, yErr)
      commands.setWalkVelocity(controlSignal[1], controlSignal[0], 0.0)
      
      self.switchWalkState()
    
    self.xErrInt += xErr
    self.yErrInt += yErr
    
    self.prevXErr = xErr
    self.prevYErr = yErr
    
    print "ball seen: ", ball.seen, "top? ", ball.fromTopCamera, "xErr: ", xErr, "yErr: ", yErr, "state: ", self.my_state
    print "front/back walk: ", controlSignal[1], "left/right walk", controlSignal[0]

class SearchGoalNode(Node):
  MY_START = 0
  MY_SUCCESS = 1
  
  MY_FIND_GOAL = 2
  
  def __init__(self):
    super(SearchGoalNode, self).__init__()
    self.myState = SearchGoalNode.MY_START
    self.yErrInt = 0.0
    self.xErrInt = 0.0
  
  def inputToWalk(self, controlInput):
    BOUNDARY_VAL = 400.0
    motor = 0.0
    if fabs(controlInput) > BOUNDARY_VAL:
      copysign(controlInput, motor)
    else:
      motor = controlInput / BOUNDARY_VAL
    return motor
  
  def switchWalkState(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    
    if not ball.seen:
      print "BALL NOT SEEN"
    else:
      if ball.fromTopCamera:
        print "BALL FROM TOP"
    
    if ball.fromTopCamera:
      yErr = 240 - ball.imageCenterY
    else:
      yErr = 80 - ball.imageCenterY
    xErr = 160 - ball.imageCenterX
    
    K_I = 0.0005
    
    FBSignal = self.inputToWalk(yErr + K_I * self.yErrInt)
    LRSignal = self.inputToWalk(xErr + K_I * self.xErrInt)
    
    print "yErr: ", yErr, "xErr: ", xErr, "front/back signal: ", FBSignal, "left/right signal ", LRSignal
    
    commands.setWalkVelocity(FBSignal, LRSignal, -pi / 50)
    
    self.yErrInt += yErr
    self.xErrInt += xErr
    
  def run(self):
    core.speech.say("searching the goal")
    
    if self.myState == SearchGoalNode.MY_SUCCESS:
      commands.stand()
      self.postSuccess()
    
    elif self.myState == SearchGoalNode.MY_START:
      commands.stand()
      self.myState = SearchGoalNode.MY_FIND_GOAL
    
    elif self.myState == SearchGoalNode.MY_FIND_GOAL:
      
      goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
      print "goal seen? ", goal.seen, "goal X: ", goal.imageCenterX

      if goal.seen and goal.imageCenterX > 145 and goal.imageCenterX < 175:
        self.myState = SearchGoalNode.MY_SUCCESS
 
      else:
        self.switchWalkState()

class KickBallNode(Node):
  """
  get ready to kick
  
  left 120, 180
  
  right 180, 180
  """
  
  MY_START = 0
  MY_GOTO_BALL = 1
  MY_READY = 2
  MY_KICK = 3
  MY_SUCCESS = 4
  
  def __init__(self):
    super(KickBallNode, self).__init__()
    self.myState = KickBallNode.MY_START
    
  def inputToWalk(self, controlInput):
    BOUNDARY_VAL = 600.0
    motor = 0.0
    if fabs(controlInput) > BOUNDARY_VAL:
      copysign(controlInput, motor)
    else:
      motor = controlInput / BOUNDARY_VAL
    return motor

  def run(self):
    core.speech.say("kicking the ball")
    
    if self.myState == KickBallNode.MY_SUCCESS:
      self.postSuccess()
      
    elif self.myState == KickBallNode.MY_START:
      commands.stand()
      self.myState = KickBallNode.MY_GOTO_BALL
    
    elif self.myState == KickBallNode.MY_READY:
      commands.stand()
      self.myState = KickBallNode.MY_KICK
    
    elif self.myState == KickBallNode.MY_KICK:
      self.myState = KickBallNode.MY_SUCCESS
    
    elif self.myState == KickBallNode.MY_GOTO_BALL:
      ball = core.world_objects.getObjPtr(core.WO_BALL)
      
      if not ball.seen:
        print "BALL NOT SEEN"
      
      if ball.fromTopCamera:
        yErr = 420 - ball.imageCenterY
      else:
        yErr = 180 - ball.imageCenterY
      xErr = 180 - ball.imageCenterX
      
      if fabs(xErr) < 15.0 and fabs(yErr) < 15.0:
        commands.stand()
        self.myState = KickBallNode.MY_READY
      else:
        LRSignal = self.inputToWalk(xErr)  # left right
        FBSignal = self.inputToWalk(yErr)  # forward backward
        
        print "xErr: ", xErr, "yErr: ", yErr, "left/right: ", LRSignal, "for/back: ", FBSignal
        
        commands.setWalkVelocity(FBSignal, LRSignal, 0.0)
    
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
    commands.setWalkVelocity(self.x, self.y, self.theta)
    if self.getTime() > self.walkTime:
      commands.stand()
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

class KickNode(Node):
  def __init__(self):
    super(KickNode, self).__init__()
    self.task = kicks.Kick()

  def run(self):
    self.task.processFrame()
    if self.task.finished():
      self.postCompleted()

