from state import * 
import commands, core, util, pose
import time

class TestMachine(StateMachine):
  def setup(self):
    start = Node()
    finish = Node()
    sit = SitNode()
    findBall = FindBallNode()
    
    self._adt(start, N, sit)
    self._adt(sit, N, findBall)
    self._adt(findBall, N, findBall)
    
class FindBallNode(Node):
  def __init__(self):
    super(FindBallNode, self).__init__()
  
  def run(self):
    ball = core.world_objects.getObjPtr(core.WO_BALL)    
    if ball.seen:
      core.speach.say("YES")
    else:
      core.speach.say("NO")

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
    self.task.processFrame() 
    if self.task.finished():
      self.postCompleted()


