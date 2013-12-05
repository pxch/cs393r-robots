from state import * 
import commands, core, util, pose
import time
import state, percepts
from math import atan2, pi, fabs
import head
import kicks

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    stand = StandNode()
    kick = KickNode()

    self._adt(start, N, stand)
    self._adt(stand, T(15.0), kick)
    self._adt(kick, C, stand)

class KickNode(Node):
  def __init__(self):
    super(KickNode, self).__init__()
    self.task = kicks.Kick()
#    self.ball_seen = False
#    self.beacon_p_y_seen = False
#    self.beacon_y_p_seen = False

  def reset(self):
    super(KickNode, self).reset()
    self.task = kicks.Kick()
  
  def run(self):
    self.task.processFrame()
    if self.task.finished():
      self.postCompleted()

#    ball = core.world_objects.getObjPtr(core.WO_BALL)
#    beacon_p_y = core.world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW)
#    beacon_y_p = core.world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)

#    print "Ball Seen? ", self.ball_seen, ", ", ball.seen

#    if self.ball_seen and self.beacon_p_y_seen and self.beacon_y_p_seen:
#      core.speech.say("Kick")
#      self.task.processFrame()
#    else:
#      if not self.ball_seen:
#        if ball.seen:
#          self.ball_seen = True
#        else:
#          core.speech.say("No ball")
#      if not self.beacon_p_y_seen:
#        if beacon_p_y.seen:
#          self.beacon_p_y_seen = True
#        else:
#          core.speech.say("No pink yellow beacon")
#      if not self.beacon_y_p_seen:
#        if beacon_y_p.seen:
#          self.beacon_y_p_seen = True
#        else:
#          core.speech.say("No yellow pink beacon")
#    if self.task.finished():
#      self.postCompleted()

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
    commands.setHeadTilt(-21.0)
    self.task.processFrame()
    if self.task.finished() and self.getTime() > 5.0:
      self.postCompleted()

