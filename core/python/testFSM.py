from state import * 
import commands, core, util, pose
import time
import state, percepts
import math
import head

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand = StandNode()
    choose = ChooseNode()
    self._adt(start, N, stand)
    self._adt(stand, C, choose)
    self._adt(choose, S(Choices.Forward), WalkNode(), S, choose)
    self._adt(choose, S(Choices.Left), TurnLeftNode(), S, choose)
    self._adt(choose, S(Choices.Right), TurnRightNode(), S, choose)
    self._adt(choose, I(4), SpeakNode('completed'), S, sit)
    self._adt(sit, C, finish)



#######################################assignment5##########################
class TestMachine5(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand = StandNode()
    scan = ScanNode()
    far_move = FarNode()
    near_move = NearNode()
    on_center = CenterNode()
    self._adt(start, N, stand)
    self._adt(stand, C, far_move)
    self._adt(far_move, S(FarNode.MY_NEAR_CENTER), scan)
    self._adt(scan, T(5.0), near_move)
    self._adt(near_move, S(NearNode.MY_FAIL), FarNode())
    self._adt(near_move, S(NearNode.MY_SUCCESS), on_center)
    self._adt(on_center, S(CenterNode.MY_OUT_FAR), FarNode())
    self._adt(on_center, S(CenterNode.MY_OUT_NEAR), NearNode())

class FarNode(Node):
  MY_START = 0
  MY_TURN = 1
  MY_FORWARD = 2
  MY_NEAR_CENTER = 3
  
  def __init__(self):
    super(FarNode, self).__init__()
    self.task = head.Scan()
    self.myState = MY_START
  
  def reset(self):
    super(FarNode, self).reset()
    self.task = head.Scan()
    self.myState = MY_START
  
  def run(self):
    self.task.processFrame()
    
    core.speech.say("Far from center")
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    
    if self.myState == MY_START:
      commands.stand()
      if robot.loc.x * robot.loc.x + robot.loc.y * robot.loc.y < 200 * 200:
        self.myState = MY_NEAR_CENTER
      else:
        angle = atan2(robot.loc.y, robot.loc.x)
        if angle > 0:
          angle = angle - pi
        elif angle < 0:
          angle = angle + pi
        else:
          if robot.orientation > 0:
            angle = pi
          else:
            angle = -pi
          
        if fabs(angle - robot.orientation) < pi / 18:
          self.myState = MY_FORWARD
        else:
          self.myState = MY_TURN
    
    elif self.myState == MY_NEAR_CENTER:
      self.posSignal(MY_NEAR_CENTER)
      return
    
    elif self.myState == MY_TURN:
      commands.setWalkVelocity(0, 0, robot.orientation - angle)
      
      angle = atan2(robot.loc.y, robot.loc.x)
      if angle > 0:
        angle = angle - pi
      elif angle < 0:
        angle = angle + pi
      else:
        if robot.orientation > 0:
          angle = pi
        else:
          angle = -pi
      
      if fabs(angle - robot.orientation) < pi / 18:
        self.myState = MY_FORWARD
            
    else:
      commands.setWalkVelocity(0.6, 0, 0)
      
      angle = atan2(robot.loc.y, robot.loc.x)
      if angle > 0:
        angle = angle - pi
      elif angle < 0:
        angle = angle + pi
      else:
        if robot.orientation > 0:
          angle = pi
        else:
          angle = -pi
      
      if fabs(angle - robot.orientation) > pi / 18:
        self.myState = MY_TURN

class ScanNode(Node):
  def __init__(self):
    super(ScanNode, self).__init__()
    self.task = head.Scan()
  
  def reset(self):
    super(ScanNode, self).reset()
    self.task = head.Scan()

  def run(self):
    self.stand()
    self.task.processFrame()
    if self.getTime() > 10.0:
      if self.task.finished():
        self.postCompleted()

class NearNode(Node):
  MY_START = 0
  MY_WALKING = 1
  MY_SUCCESS = 2
  MY_FAIL = 3
  
  def __init__(self):
    super(NearNode, self).__init__()
    self.task = head.Scan(2.0, 5.0)
    self.myState = MY_START
       
  def reset(self):
    super(NearNode, self).reset()
    self.task = head.Scan(2.0, 5.0)
    self.myState = MY_START
  
  def run(self):
    core.speech.say("Near the center")
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    
    if myState == MY_START:
      if robot.loc.x * robot.loc.x + robot.loc.y * robot.loc.y < 50 * 50:
        self.myState = MY_SUCCESS
      else:
        angle = atan2(robot.loc.y, robot.loc.x)
        if angle > 0:
          angle = angle - pi
        elif angle < 0:
          angle = angle + pi
        else:
          if robot.orientation > 0:
            angle = pi
          else:
            angle = -pi
    elif myState == MY_SUCCESS:
      self.postSignal(MY_SUCCESS)
      return
    elif myState == MY_FAIL:
      self.postSignal(MY_FAIL)
      return
    else:
      angle = atan2(robot.loc.y, robot.loc.x)
      if angle > 0:
        angle = angle - pi
      elif angle < 0:
        angle = angle + pi
      else:
        if robot.orientation > 0:
          angle = pi
        else:
          angle = -pi
      
      commands.setWalkVelocity(0.3, 0, robot.orientation - angle)
      
      if robot.loc.x * robot.loc.x + robot.loc.y * robot.loc.y < 50 * 50:
        self.myState = MY_SUCCESS
      elif robot.loc.x * robot.loc.x + robot.loc.y * robot.loc.y > 200 * 200:
        self.myState = MY_FAIL
  
class CenterNode(Node):
  MY_START = 0
  MY_KEEPING = 1
  MY_OUT_FAR = 2
  MY_OUT_NEAR = 3
  
  def __init__(self):
    super(CenterNode, self).__init__()
    self.task = head.Scan(2.0, 5.0)
    self.myState = MY_START
  
  def reset(self):
    super(CenterNode, self).reset()
    self.task = head.Scan(2.0, 5.0)
    self.myState = MY_START

  def run(self):
    core.speech.say("On center")
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    
    if self.myState == MY_START:
      self.myState = MY_KEEPING
    elif self.myState == MY_KEEPING:
      commands.setWalkVelocity(0, 0, 0)
      if robot.loc.x * robot.loc.x + robot.loc.y * robot.loc.y > 200 * 200:
        self.myState = MY_OUT_FAR
      elif robot.loc.x * robot.loc.x + robot.loc.y * robot.loc.y > 50 * 50:
        self.myState = MY_OUT_NEAR
    elif self.myState == MY_OUT_FAR:
      self.postSignal(MY_OUT_FAR)
      return
    else:
      self.postSignal(MY_OUT_NEAR)
      return
    
class towardsToCenterNode(Node):
  def __init__(self):
    super(towardsToCenterNode, self).__init__()

  def run(self):
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    
    if abs(robot.loc.x) < 50 and abs(robot.loc.y) < 50:
      self.postCompleted()

    elif robot.loc.x >=0 and robot.loc.y >=0:
      if abs(tan(robot.orientation) - y/x) < 0.1 and robot.orientation >= 180 and robot.orientation <= 270:
        commands.setWalkVelocity(0.5, 0, 0) 	      
      else:
	commands.setWalkVelocity(0, 0, -0.25)

    elif robot.loc.x <= 0 and robot.loc.y >= 0:
      if abs(tan(robot.orientation) - y/x) < 0.1 and robot.orientation >= 270 and robot.orientation <= 360:
        commands.setWalkVelocity(0.5, 0, 0) 	      
      else:
	commands.setWalkVelocity(0, 0, -0.25)

    elif robot.loc.x <= 0 and robot.loc.y <= 0:
      if abs(tan(robot.orientation) - y/x) < 0.1 and robot.orientation >= 0 and robot.orientation <= 90:
        commands.setWalkVelocity(0.5, 0, 0) 	      
      else:
	commands.setWalkVelocity(0, 0, -0.25)

    elif robot.loc.x >= 0 and robot.loc.y <= 0:
      if abs(tan(robot.orientation) - y/x) < 0.1 and robot.orientation >= 90 and robot.orientation <= 180:
        commands.setWalkVelocity(0.5, 0, 0) 	      
      else:
	commands.setWalkVelocity(0, 0, -0.25)


class WalkNode(Node):
  def run(self):
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    print robot.loc.x
    print robot.loc.y
    print robot.orientation
    print "one"
   
    if self.getTime() > 30.0:
      commands.stand()
      self.postSuccess()



################################################################################	

def rand():
  t = int(time.time() * 1000) % 1000
  return t

class Choices:
  Left = 0
  Right = 1
  Forward = 2
  NumChoices = 3

class ChooseNode(Node):
  _choices = 0
  def run(self):
    if ChooseNode._choices > 10:
      self.poseCompletion()
      return
    if self.getTime() > 4.0:
      choice = (rand() % Choices.NumChoices)
      self.postSignal(choice)
      ChooseNode._choices += 1

class TurnLeftNode(Node):
  def run(self):
    commands.setWalkVelocity(0, 0, .25)
    if self.getTime() > 4.0:
      commands.stand()
      self.postSuccess()

class TurnRightNode(Node):
  def run(self):
    commands.setWalkVelocity(0, 0, -.25)
    if self.getTime() > 4.0:
      commands.stand()
      self.postSuccess()

class SpeakNode(Node):
  def __init__(self, phrase):
    super(SpeakNode, self).__init__()
    self.phrase = phrase

  def run(self):
    if self.getFrames() == 0:
      core.speech.say(self.phrase)
    if self.getTime() > 4.0:
      self.postSuccess()

class WalkNode(Node):
  def run(self):
    commands.setWalkVelocity(.5, 0, 0)
    if self.getTime() > 2.0:
      commands.stand()
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
