from state import * 
import commands, core, util, pose, percepts, kicks
import time
from math import pi, fabs, copysign, atan
from re import search
from kicks import Kick

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
    dribble = DribbleNode()
    ballInGoal = GoalInBallNode()

    self._adt(start, N, TurnHeadNode(0.0, 1.0, False), C, TiltHeadNode(-26.5), C, stand)
    self._adt(stand, C, searchBall)
    self._adt(searchBall, S, searchGoal)
    self._adt(searchGoal, S(SearchGoalNode.MY_SUCCESS), dribble)
    self._adt(searchGoal, S(SearchGoalNode.MY_BALL_LOST), searchBall)
    self._adt(dribble, S(DribbleNode.MY_SUCCESS), readyToKick)
    self._adt(dribble, S(DribbleNode.MY_BALL_LOST), searchBall)
    self._adt(readyToKick , S(KickBallNode.MY_SUCCESS), kick)
    self._adt(kick, S(KickBallNode.MY_BALL_LOST), searchBall)
    self._adt(kick, C, ballInGoal)
    self._adt(ballInGoal, S(GoalInBallNode.SIG_SCORED_NO), searchBall)
    self._adt(ballInGoal, S(GoalInBallNode.SIG_SCORED_YES), sit)
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
  
  def reset(self):
    super(SearchBallNode, self).reset()
    self.myState = SearchBallNode.MY_START
  
  def __init__(self):
    super(SearchBallNode, self).__init__()
    self.my_state = SearchBallNode.MY_START 
    
    # error = expected_location - current_location
    self.xErrInt = 0.0  # integral of x error
    self.yErrInt = 0.0  # integral of y error
    
    self.prevXErr = None
    self.prevYErr = None
  
  def PID(self, xErr, yErr):
    # K_I = 0.001
    # K_D = 0.0

    # dErrDt = self.dErrDt(xErr, yErr)
    #
    # xInput = xErr + K_I * self.xErrInt - K_D * dErrDt[0]
    #
    # yInput = yErr + K_I * self.yErrInt - K_D * dErrDt[1]
    #
    # return (self.inputToMotor(xInput), self.inputToMotor(yInput))
    if xErr > 0:
      xInput = 0.5
    else:
      xInput = -0.5
    if yErr > 0:
      yInput = 0.5
    else:
      yInput = -0.5
    return (xInput, yInput)
  
  def inputToMotor(self, inValue):
    BOUNDARY_VAL = 400.0
    motorSpeed = 0
    if fabs(inValue) > BOUNDARY_VAL:
      if inValue > 0:
        motorSpeed = 1
      else:
        motorSpeed = -1
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
      
      if ball.imageCenterY < 50:  # far
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
  
  MY_BALL_LOST = 3
  
  def reset(self):
    super(SearchGoalNode, self).reset()
    self.myState = SearchGoalNode.MY_START
  
  def __init__(self):
    super(SearchGoalNode, self).__init__()
    self.myState = SearchGoalNode.MY_START
    self.yErrInt = 0.0
    self.xErrInt = 0.0
    
    self.turnDirection = None
  
  def inputToWalk(self, controlInput):
    BOUNDARY_VAL = 200.0
    motor = 0.0
    if fabs(controlInput) > BOUNDARY_VAL:
      if inValue > 0:
        motorSpeed = 1
      else:
        motorSpeed = -1
    else:
      motor = controlInput / BOUNDARY_VAL
    return motor
  
  def switchWalkState(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    
    if not ball.seen:
      print "BALL NOT SEEN"
      self.postSignal(SearchGoalNode.MY_BALL_LOST)
      return
    else:
      if ball.fromTopCamera:
        print "BALL FROM TOP"
    
    if ball.fromTopCamera:
      yErr = 240 - ball.imageCenterY
    else:
      yErr = 80 - ball.imageCenterY
    xErr = 160 - ball.imageCenterX
    
    K_I = 0.001
    
    # FBSignal = self.inputToWalk(yErr + K_I * self.yErrInt)
    # LRSignal = self.inputToWalk(xErr + K_I * self.xErrInt)
    if xErr > 0:
      LRSignal = 0.3
    else:
      LRSignal = -0.3
    if yErr > 0:
      FBSignal = 0.3
    else:
      FBSignal = -0.3
    
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    if goal.seen:
      if self.turnDirection == None:
        self.turnDirection = -1.0  # turn right while going left, goal on right
        if goal.imageCenterX < 160.0:
          self.turnDirection = 1.0  # goal on left, turn left while going right
    else:
      self.turnDirection = -1.0
    
    print "yErr: ", yErr, "xErr: ", xErr, "front/back signal: ", FBSignal, "left/right signal ", LRSignal, "turn", self.turnDirection
    
    commands.setWalkVelocity(FBSignal, LRSignal, self.turnDirection * pi / 50.0)
    
    self.yErrInt += yErr
    self.xErrInt += xErr
    
  def run(self):
    core.speech.say("searching the goal")
    
    if self.myState == SearchGoalNode.MY_SUCCESS:
      commands.stand()
      self.postSignal(SearchGoalNode.MY_SUCCESS)
    
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
  MY_BALL_LOST = 5
  
  def reset(self):
    super(KickBallNode, self).reset()
    self.myState = KickBallNode.MY_START
  
  def __init__(self):
    super(KickBallNode, self).__init__()
    self.myState = KickBallNode.MY_START
    
    self.xErrInt = 0.0
    self.yErrInt = 0.0
    
  def inputToWalk(self, controlInput):
    BOUNDARY_VAL = 800.0
    motor = 0.0
    if fabs(controlInput) > BOUNDARY_VAL:
      if controlInput > 0:
        motor = 1
      else:
        motor = -1
    else:
      motor = controlInput / BOUNDARY_VAL
    return motor

  def run(self):
    core.speech.say("kicking the ball")
    
    if self.myState == KickBallNode.MY_SUCCESS:
      self.postSignal(KickBallNode.MY_SUCCESS)
      
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
        self.postSignal(KickBallNode.MY_BALL_LOST)
        return
      
      if ball.fromTopCamera:
        yErr = 240 + 190 - ball.imageCenterY
      else:
        yErr = 190 - ball.imageCenterY
      xErr = 190 - ball.imageCenterX
      
      if fabs(xErr) < 5.0 and fabs(yErr) < 5.0:
        commands.stand()
        self.myState = KickBallNode.MY_READY
      else:
        K_I = 0.001
        
        # Bang-Bang Control
        if xErr > 0:
          LRSignal = 0.2
        else:
          LRSignal = -0.2
    
        if yErr > 0:
          FBSignal = 0.2
        else:
          FBSignal = -0.2
        
        # PID Control
        # LRSignal = self.inputToWalk(xErr + K_I * self.xErrInt)  # left right
        # FBSignal = self.inputToWalk(yErr + K_I * self.yErrInt)  # forward backward
        
        print "xErr: ", xErr, "yErr: ", yErr, "left/right: ", LRSignal, "for/back: ", FBSignal
        
        commands.setWalkVelocity(FBSignal, LRSignal, 0.0)
        
        self.xErrInt += xErr
        self.yErrInt += yErr
        
class DribbleNode(Node):
  """
  dribble coordinate
  
  160 200
  """
  
  MY_START = 0
  
  MY_SUCCESS = 2
  
  MY_TURNING = 3
  MY_MOVING = 4
  
  MY_BALL_LOST = 5
  
  MY_MOVING_MAX = 50
  
  def reset(self):
    super(DribbleNode, self).reset()
    self.myState = DribbleNode.MY_START
  
  def __init__(self):
    super(DribbleNode, self).__init__()
    self.myState = DribbleNode.MY_START
    
    self.movingCounter = 0
    
    self.xErrInt = 0.0
    self.yErrInt = 0.0
  
  def toMotor(self, inputVal):
    BOUNDARY_VAL = 800.0
    motor = 0.0
    if fabs(inputVal) > BOUNDARY_VAL:
      if inputVal > 0:
        motor = 1
      else:
        motor = -1
    else:
      motor = inputVal / BOUNDARY_VAL
    return motor

  def ballSignal(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    
    K_I = 0.001
    
    if not ball.seen:
      print "BALL NOT SEEN"
      self.postSignal(DribbleNode.MY_BALL_LOST)
      return None
        
    xErr = 160.0 - ball.imageCenterX
    if ball.fromTopCamera:
      yErr = 440.0 - ball.imageCenterY
    else:
      yErr = 200.0 - ball.imageCenterY 
    
    # Bang-Bang Control
    if xErr > 0:
      LRSignal = 0.2
    else:
      LRSignal = -0.2
     
    if yErr > 0:
      FBSignal = 0.2
    else:
      FBSignal = -0.2
    
    # PID Control
    
    # LRSignal = self.toMotor(xErr + K_I * self.xErrInt)
    # FBSignal = self.toMotor(yErr + K_I * self.yErrInt)
    
    print "xErr: ", xErr, "yErr", yErr, "left/right ", LRSignal, "for/back ", FBSignal
    
    goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
    print "goal seen? ", goal.seen, "blue ratio: ", goal.radius
    
    self.xErrInt += xErr
    self.yErrInt += yErr
    
    return (FBSignal, LRSignal)
  
  def run(self):
    if self.myState == DribbleNode.MY_START:
      commands.stand()
      self.myState = DribbleNode.MY_MOVING
      
    elif self.myState == DribbleNode.MY_SUCCESS:
      commands.stand()
      self.postSignal(DribbleNode.MY_SUCCESS)
    
    elif self.myState == DribbleNode.MY_TURNING:
      print "dribble turning"
      
      goal = core.world_objects.getObjPtr(core.WO_OPP_GOAL)
      
      if not goal.seen:
        print "GOAL NOT SEEN!!!"
      
      if fabs(goal.imageCenterX - 160.0) < 15.0:
        if goal.radius > 0.28:
          self.myState = DribbleNode.MY_SUCCESS
        else:
          self.myState = DribbleNode.MY_MOVING
      
      else:
        if goal.imageCenterX > 160.0:
          turnDirection = -1.0  # goal on right, turn right while going left
        else:
          turnDirection = 1.0
          
        ballSignal = self.ballSignal()
        if ballSignal == None:
          return
        commands.setWalkVelocity(ballSignal[0], ballSignal[1], turnDirection * pi / 70.0)
      
    elif self.myState == DribbleNode.MY_MOVING:
      print "dribble moving"
      
      self.movingCounter += 1
      
      ballSignal = self.ballSignal()
      
      commands.setWalkVelocity(ballSignal[0], ballSignal[1], 0.0)
    
      line = core.world_objects.getObjPtr(core.WO_OPP_GOAL_LINE)
      if line.fieldLineIndex == 2 or line.fieldLineIndex == 3:
        self.myState = self.MY_SUCCESS

      if self.movingCounter == DribbleNode.MY_MOVING_MAX:
        self.movingCounter = 0
        self.myState = self.MY_TURNING
        
class GoalInBallNode(Node):
  MY_START = 0
  
  FOLLOW_BALL = 1
  
  TIME_OUT = 15.0
  
  """
  signals to send out
  """
  SIG_SCORED_YES = 3
  SIG_SCORED_NO = 4
  
  def reset(self):
    super(GoalInBallNode, self).reset()
    self.myState = GoalInBallNode.MY_START
  
  def __init__(self):
    super(GoalInBallNode, self).__init__()
    
    self.myState = GoalInBallNode.MY_START
    self.scored = False
  
  def run(self):
    if self.myState == GoalInBallNode.MY_START:
      commands.stand()
      self.myState = GoalInBallNode.FOLLOW_BALL
    
    elif self.myState == GoalInBallNode.FOLLOW_BALL:
      ball = core.world_objects.getObjPtr(core.WO_BALL)
      if ball.type == 1:
        self.scored = True
      
      print "scored? ", self.scored, "time", self.getTime()
      
      if self.getTime() > GoalInBallNode.TIME_OUT:
        if self.scored:
          self.postSignal(GoalInBallNode.SIG_SCORED_YES)
        else:
          self.postSignal(GoalInBallNode.SIG_SCORED_NO)
      
    
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
      self.postCompleted()

class KickNode(Node):
  def __init__(self):
    super(KickNode, self).__init__()
    self.task = kicks.Kick()

  def reset(self):
    super(KickNode, self).reset()
    self.task = kicks.Kick()
  
  def run(self):
    self.task.processFrame()
    if self.task.finished():
      self.postCompleted()

