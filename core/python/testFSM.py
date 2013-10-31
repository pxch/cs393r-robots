from state import * 
import commands, core, util, pose
import time
import state, percepts
from math import pi
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
    walk = WalkNode()
    walk1 = towardsToCenterNode()
    self._adt(start, N, stand)
    self._adt(stand, C, scan)
    self._adt(scan, C, sit)
    self._adt(sit, C, finish)


class ScanNode(Node):
  def __init__(self):
    super(ScanNode, self).__init__()
    self.task = head.Scan(2.0, 3.0, 30)
  
  def reset(self):
    super(ScanNode, self).reset()
    self.task = head.Scan(2.0, 3.0, 30)

  def run(self):
    self.task.processFrame()
    commands.setWalkVelocity(0.3, 0, 0)
    if self.getTime() > 30.0:
      if self.task.finished():
        self.postCompleted()

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
