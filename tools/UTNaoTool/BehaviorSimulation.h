#ifndef BEHAVIOR_SIM_H
#define BEHAVIOR_SIM_H

#ifdef TOOL
#include <QString>
#endif

#include <vector>
#include <string>

#include <math/Pose2D.h>
#include <math/Geometry.h>
#include <common/WorldObject.h>
#include <localization/LocalizationModule.h>
#include "SimulatedPlayer.h"

class WorldObjectBlock;
class GameStateBlock;
class FrameInfoBlock;

class WorldObject;
class VisionCore;
class Memory;

class BehaviorSimulation {

 public:
  BehaviorSimulation(int nplayers, bool penaltyKick, bool locMode);
  ~BehaviorSimulation();

  void setPenalty(int index);

  Memory* getMemory(int id);
  std::vector<std::string> getTextDebug(int id);
  void changeSimulationState(int state);
  void moveRobot(int index, AngRad rotation, Point2D movement);
  void moveBall(Point2D translation);
  void changeSimulationKickoff();
  void restartSimulationLua();
  void setStrategy();
  void doPenaltyKickReset();
  void simulationStep();
  void setFallen(int index);
  void flipRobot(int index);

  void stepBall(Point2D &ballLoc, Point2D &ballVel);
  void stepTimer(Point2D &ballLoc, Point2D &ballVel);
  void stepCheckGoal(Point2D &ballLoc, Point2D &ballVel);
  void stepCheckBounds(Point2D &ballLoc, Point2D &ballVel);
  void stepCheckBallCollisions(Point2D &ballLoc, Point2D &ballVel);
  void stepPlayerFallen(int i);
  void stepPlayerBounds(int i);
  void stepPlayerKick(int i);
  void stepPlayerBumpBall(int i, Point2D &ballLoc, Point2D &ballVel, WorldObject *robot);
  void stepPlayerPenaltyBox(int i, WorldObject *robot);
  void stepPlayerCollisions(int i, WorldObject *robot);
  void stepPlayerComm(int i);

  void getTeamSignAndSelf(int i, int &teamsign, int &self);
  void setObjectFromPose(int i, int teamsign, const Pose2D *pose);

  void restartLua();

  #ifdef TOOL
  QString getSimInfo();
  QString simInfo;
  #endif

  // for comparing behavior params
  //void compareParams();
  //void differParams(int param);
  //void differBHumanParams(int param);
  void runParamTest();
  void runParamTests();

  int checkLocalizationErrors();
  int measureLocalizationErrors(int redParam);
  void runKickTests();

  WorldObject* getPlayerObject(int id);

  SimulatedPlayer* sims[WO_OPPONENT_LAST+1];
  bool simActive[WO_OPPONENT_LAST+1];
  float fallenTime[WO_OPPONENT_LAST+1];
  int lastKick;
  float lastKickX;

  // sim variables
  int currentSim;
  int simBlueScore;
  int simRedScore;
  float simTimer;
  float halfTimer;
  bool simPenaltyKick;
  bool ballClearedFromCircle;
  bool simOn;
  int nplayers;
  bool locMode;
  float timeInc;
  int numHalves;
  bool PRINT;
  bool forceManualPositions;
  bool forceDesiredPositions;

  void kickBall();
  void setSimScore(bool blue);

  Memory* memory_;
  WorldObjectBlock* worldObjects;
  GameStateBlock* gameState;
  FrameInfoBlock* frameInfo;

 private:

};


#endif
