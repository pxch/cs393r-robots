from state import * 
import commands, core, util, pose
import time
import state, percepts
from math import atan2, pi, fabs
import head

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand = StandNode()
    searchball = SearchBallNode()
    walktoball = WalktoBallNode()
    searchgoal = SearchGoalNode()
    kickball = KickBallNode()
    
    self._adt(start, N, stand)
    self._adt(stand, C, searchball)
    self._adt(searchball, S(SearchBallNode.MY_SUCCESS), walktoball)
    self._adt(searchball, S(SearchBallNode.MY_NOGOAL), searchgoal)
    self._adt(searchball, S(SearchBallNode.MY_NOBALL), sit)
    self._adt(searchgoal, S(SearchGoalNode.MY_SUCCESS), walktoball)
    self._adt(searchgoal, S(SearchGoalNode.MY_LOSTBALL), searchball)
    self._adt(walktoball, S(WalktoBallNode.MY_SUCCESS), kickball)
    self._adt(walktoball, S(WalktoBallNode.MY_LOSTBALL), searchball)
    self._adt(walktoball, S(WalktoBallNode.MY_LOSTGOAL), searchgoal)
    self._adt(kickball, S(KickBallNode.MY_SUCCESS), searchball)
    self._adt(kickball, S(KickBallNode.MY_FAIL), kick)
    self._adt(sit, C, finish)

class SearchBallNode(Node):
  MY_SUCCESS = 1
  MY_NOGOAL = 2
  MY_NOBALL = 3

  def __init__(self):
    super(SearchBallNode, self).__init__()
    
  def reset(self):
    super(SearchBallNode, self).reset()

  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)
    goal = core.world_objects.getObjPrt(core.WO_BEACON_YELLOW_PINK)
    if ball.seen and goal.seen:
      self.postSignal(SearchBallNode.MY_SUCCESS)
      return
    elif ball.seen and not goal.seen:
      self.postSignal(SearchBallNode.MY_NOGOAL)
      return
    else:
      commands.setWalkVelocity(0, 0, - pi / 18)
      if self.getTime() > 5.0:
        commands.stand()

class SearchGoalNode(Node):
  MY_SUCCESS = 1
  MY_LOSTBALL = 2

  def __init__(self):
    super(SearchGoalNode, self).__init__()
    
  def reset(self):
    super(SearchGoalNode, self).reset()

  def run(self):
    commands.stand();

class WalktoBallNode(Node):
  MY_SUCCESS = 1
  MY_LOSTBALL = 2
  MY_LOSTGOAL = 3

  def __init__(self):
    super(WalktoBallNode, self).__init__()
    
  def reset(self):
    super(WalktoBallNode, self).reset()

  def run(self):
    commands.stand();

class KickBallNode(Node):
  MY_SUCCESS = 1
  MY_FAIL = 2

  def __init__(self):
    super(KickBallNode, self).__init__()
    
  def reset(self):
    super(KickBallNode, self).reset()

  def run(self):
    commands.stand();

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
    if self.task.finished() and self.getTime() > 5.0:
      self.postCompleted()
