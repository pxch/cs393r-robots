from state import * 
import commands, core, util, pose
import time
from math import pi

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


#######################################assignment4###########################

class TestMachine4(StateMachine):  #detect ball
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    turnleft = HeadTurnLeftNode()
    turnright = HeadTurnRightNode()
    turn = HeadTurnNode()

    detectBeacons = DetectBeaconsNode()
    self._adt(start, N, detectBeacons)
    self._adt(detectBeacons, S, sit)
    self._adt(sit, C, finish)


class DetectBeaconsNode(Node):
  def __init__(self):
    super(DetectBeaconsNode, self).__init__()
    self._time = 0
  
  def run(self):
    self._time = self._time + 1
    commands.setWalkVelocity(0.1, 0, 0)  #turn left

    if self._time%12 >= 0 and self._time%12 < 6:
      commands.setHeadPan(1, 6.0)

    else:
      commands.setHeadPan(-1, 6.0)
    
    if self.getTime() > 40.0:
      self.postSuccess()


#######################################assignment5##########################
class TestMachine5(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand = StandNode()
    walk = WalkNode()
    walk1 = towardsToCenterNode()
    self._adt(start, N, stand)
    self._adt(stand, C, walk)
    self._adt(walk, C, sit)
    self._adt(sit, C, finish)

class towardsToCenterNode(Node):
  def __init__(self):
    super(towardsToCenterNode, self).__init__()

  def run(self):
    robot = core.world_objects.getObjPtr(core.robot_state.WO_SELF)
    
    if robot.loc.x == 0 and robot.loc.y == 0:
      self.postCompleted()
    else: 
      commands.setWalkVelocity(0.2, 0, robot.orientation)

class WalkNode(Node):
  def run(self):
    commands.setWalkVelocity(.5, 0, -pi/12)
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
