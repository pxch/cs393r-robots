#ifndef SIM_PLAYER_H
#define SIM_PLAYER_H

#include <vector>
#include <string>

#include <math/Pose2D.h>
#include <math/Geometry.h>
#include <common/RobotInfo.h>

class BehaviorBlock;
class CameraBlock;
class FrameInfoBlock;
class GameStateBlock;
class JointBlock;
class KickRequestBlock;
class OdometryBlock;
class RobotStateBlock;
class SensorBlock;
class ALWalkParamBlock;
class WalkRequestBlock;
class WorldObjectBlock;
class TeamPacketsBlock;
class OpponentBlock;
class JointCommandBlock;
class LocalizationBlock;
class ProcessedSonarBlock;
class WalkInfoBlock;
class BehaviorParamBlock;

class VisionCore;
class Memory;

class SimulatedPlayer {

 public:
  SimulatedPlayer(int team, int self, int index, bool lMode);
  ~SimulatedPlayer();

  void init(int team, int self, Memory* memory);

  bool processFrame(WorldObjectBlock* wo, GameStateBlock* gs);
  void setMemory(Memory* memory);
  Memory* getMemory();
  void updateMemoryBlocks();
  std::vector<std::string> getTextDebug();
  void changeState(int state);
  void moveRobot(int index, AngRad rotation, Point2D movement);
  void moveBall(Point2D translation);
  void changeKickoff();
  void restartLua();
  void setPenalty(WorldObjectBlock* simMem);
  void setPenaltyPosition(WorldObjectBlock* simMem);
  void setFallen();
  void setStrategy();
  void resetCounters();

  VisionCore* core;
  Memory* memory_;
  int team_;
  int self_;
  int index_;
  float penaltySeconds;
  float kickSeconds;
  float kickHitSeconds;
  float getupSeconds;
  int diving;
  float roll;
  float dt;
  bool locMode;
  
  // noise factors
  float visionErrorFactor;//0.0;
  float missedObsFactor;//0.0;
  float odometryErrorFactor;
  float kickErrorFactor;
  
  // kick, walk, getup speeds
  float getupTimeLength;
  float kickFullTime;
  float kickHitTime;
  float maxFwdVel;
  float maxSideVel;
  float maxTurnVel;

  // to target
  bool walkToTarget;
  Point2D target;

  // walk info
  Pose2D absWalkVel;
  Pose2D relWalkVel;

  float panStopTime;
  bool panMoving;

  // specific memory blocks
  OpponentBlock* opponentMem;
  WorldObjectBlock* worldObjects;
  SensorBlock* sensors;
  TeamPacketsBlock* teamPackets;
  FrameInfoBlock* frameInfo;
  RobotStateBlock* robotState;
  GameStateBlock* gameState;
  BehaviorBlock* behavior;
  WalkRequestBlock* walkRequest;
  KickRequestBlock* kickRequest;
  ALWalkParamBlock* walkParams;
  JointCommandBlock* jointCommands;
  JointBlock* jointValues;
  LocalizationBlock* localizationMem;
  ProcessedSonarBlock* processedSonar;
  WalkInfoBlock* walkInfo;
  OdometryBlock* odometry;
  BehaviorParamBlock* behaviorParams;

  bool PRINT;

  static bool DEBUGGING_POSITIONING;

 private:


  void updateBasicInputs(WorldObjectBlock* simulationMem, GameStateBlock *gameState);
  void updateGroundTruthInputs(WorldObjectBlock* simulationMem);
  void updateFakeVisionInputs(WorldObjectBlock* simulationMem);
  bool updateOutputs(WorldObjectBlock* simulationMem);

  float crop(float v, float min, float max);
  ImageParams iparams_;

};


#endif
