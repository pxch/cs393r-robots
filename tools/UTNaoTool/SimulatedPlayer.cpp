#include "SimulatedPlayer.h"

// core
#include <VisionCore.h>

#include <localization/LocalizationModule.h>
#include <communications/CommunicationModule.h>
#include <opponents/OppModule.h>

// memory
#include <memory/BehaviorBlock.h>
#include <memory/CameraBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/JointBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/Memory.h>
#include <memory/TeamPacketsBlock.h>
#include <memory/OpponentBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/ALWalkParamBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/JointBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/ProcessedSonarBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/BehaviorParamBlock.h>

#include <stdlib.h>

// lua
#include <lua/LuaModule.h>

bool SimulatedPlayer::DEBUGGING_POSITIONING = false;

SimulatedPlayer::SimulatedPlayer(int team, int self, int index, bool lMode) : iparams_(Camera::TOP) {
  team_ = team;
  self_ = self;
  index_ = index;
  penaltySeconds = 0;
  kickSeconds = 0;
  kickHitSeconds = 0;
  getupSeconds = 0;
  diving = Dive::NONE;
  walkToTarget = false;
  locMode = lMode;
  dt = 1.0/30.0;

  panStopTime = 0;
  panMoving = false;

  PRINT = false;

  // noise factors (vision, odometry, kicking)
  visionErrorFactor = 1.0;//0.0;
  missedObsFactor = 1.0;//0.0;
  odometryErrorFactor = 1.0;
  kickErrorFactor = 1.0;
  
  // make red team have lots of error
  /*
  if (false && team_ == TEAM_RED){
    visionErrorFactor = 2.0;
    missedObsFactor = 2.0;
    odometryErrorFactor = 2.0;
    kickErrorFactor = 2.0;
  } else { // and blue have less error
    visionErrorFactor = 0.5;
    missedObsFactor = 0.5;
    odometryErrorFactor = 0.5;
    kickErrorFactor = 0.5;
  } 
  */

  visionErrorFactor = 1.5;
  missedObsFactor = 1.5;

  // params of how long things take
  getupTimeLength = 12.0; // get up is 12 seconds
  kickFullTime = 2.1;   // kick takes 2.1 seconds
  kickHitTime = 1.4;      // 1.4 seconds from start of kick to kick movement
  maxFwdVel = 240.0;
  maxSideVel = 120.0;
  maxTurnVel = 130.0 * DEG_T_RAD;

  // init vision core
  core = new VisionCore(CORE_TOOLSIM,false,team_,self_);

  // new memory
  memory_ = core->memory_;

  srand(time(NULL));

  updateMemoryBlocks();

  robotState->team_ = team_;
  robotState->WO_SELF = self_;
  robotState->role_ = self_;
  worldObjects->init(team_);
  gameState->ourKickOff = (team_ == TEAM_BLUE);
  robotState->robot_id_ = -1;

  // set no opponents
  for (int i = 0; i < MAX_OPP_MODELS_IN_MEM; i++){
    opponentMem->alpha[i] = -1;
  }

  // turn on text logging
  core->textlog_.toolSimMode = true;
  restartLua();
}

SimulatedPlayer::~SimulatedPlayer(){
  delete core;
  // core will delete memory for us
  core = NULL;
  memory_ = NULL;
}

// start the simulation from the last real snapshot
void SimulatedPlayer::setMemory(Memory* memory){
  core->updateMemory(memory);
  if (core->lua_){
    cout << "updating lua memory" << endl;
    core->lua_->updateModuleMemory(memory);
  }
  if (core->communications_){
    core->communications_->updateModuleMemory(memory_);
  }
  core->lua_->startLua();
}

Memory* SimulatedPlayer::getMemory(){
  return memory_;
}

void SimulatedPlayer::updateMemoryBlocks(){

  opponentMem = NULL;
  worldObjects = NULL;
  sensors = NULL;
  teamPackets = NULL;
  frameInfo = NULL;
  robotState = NULL;
  gameState = NULL;
  behavior = NULL;
  walkRequest = NULL;
  walkInfo = NULL;
  kickRequest = NULL;
  walkParams = NULL;
  jointCommands = NULL;
  jointValues = NULL;
  localizationMem = NULL;
  processedSonar = NULL;
  odometry = NULL;
  behaviorParams = NULL;

  absWalkVel = Pose2D(0,0,0);
  relWalkVel = Pose2D(0,0,0);


  memory_->getOrAddBlockByName(localizationMem, "localization");
  memory_->getOrAddBlockByName(opponentMem, "opponents");
  memory_->getOrAddBlockByName(worldObjects, "world_objects");
  memory_->getOrAddBlockByName(sensors, "vision_sensors");
  memory_->getOrAddBlockByName(teamPackets, "team_packets");
  memory_->getOrAddBlockByName(frameInfo, "vision_frame_info");
  memory_->getOrAddBlockByName(robotState, "robot_state");
  memory_->getOrAddBlockByName(gameState, "game_state");
  memory_->getOrAddBlockByName(behavior, "behavior");
  memory_->getOrAddBlockByName(walkRequest, "vision_walk_request");
  memory_->getOrAddBlockByName(kickRequest, "vision_kick_request");
  memory_->getOrAddBlockByName(walkParams, "vision_al_walk_param");
  memory_->getOrAddBlockByName(jointCommands, "vision_joint_commands");
  memory_->getOrAddBlockByName(jointValues, "vision_joint_angles");
  memory_->getOrAddBlockByName(processedSonar, "vision_processed_sonar");
  memory_->getOrAddBlockByName(walkInfo, "vision_walk_info");
  memory_->getOrAddBlockByName(odometry, "vision_odometry");
  memory_->getOrAddBlockByName(behaviorParams, "behavior_params");

  // set the lua pointers again in case there were modules that didn't exist until this last call
  frameInfo->source = MEMORY_SIM;
  if (core->lua_){
    cout << "updating lua memory" << endl;
    core->lua_->updateModuleMemory(memory_);
  }
  if (core->communications_){
    core->communications_->updateModuleMemory(memory_);
  }
  core->lua_->startLua();
}

bool SimulatedPlayer::processFrame(WorldObjectBlock* simulationMem, GameStateBlock* simulationState){

  updateBasicInputs(simulationMem, simulationState);
  if (locMode)
    updateFakeVisionInputs(simulationMem);
  else
    updateGroundTruthInputs(simulationMem);

  // run localization
  if (locMode){
    core->localization_->processFrame();
    core->opponents_->processFrame();
  } else {
    // set ball in keepRelBallPos
    WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
    WorldObject* robot = &(worldObjects->objects_[self_]);
    behavior->keeperRelBallPos = ball->loc.globalToRelative(robot->loc,robot->orientation);
    behavior->keeperRelBallVel = ball->absVel.globalToRelative(Point2D(0,0),robot->orientation);
  }

  // call behavior process frame
  //disabled for 393r-f2013 core->localization_->filterCloseBallPosition(-140,0,1.0);
  core->lua_->behaviorProcessFrame();
  core->communications_->processFrame();

  return updateOutputs(simulationMem);

}

void SimulatedPlayer::updateBasicInputs(WorldObjectBlock* simulationMem, GameStateBlock* simulationState){

  // clear text debug
  core->textlog_.textEntries.clear();

  // update game state from simulation mem
  if (gameState->state != PENALISED)
    gameState->state = simulationState->state;
  gameState->secsRemaining = simulationState->secsRemaining;
  if (team_ == TEAM_BLUE) gameState->ourKickOff = simulationState->ourKickOff;
  else gameState->ourKickOff = !simulationState->ourKickOff;
  gameState->isPenaltyKick = simulationState->isPenaltyKick;

  if (team_ == TEAM_BLUE){
    gameState->ourScore = simulationState->ourScore;
    gameState->opponentScore = simulationState->opponentScore;
  } else {
    gameState->opponentScore = simulationState->ourScore;
    gameState->ourScore = simulationState->opponentScore;
  } 

  if (gameState->state != PENALISED && gameState->state != PLAYING)
    penaltySeconds = -dt;

  // check if penalty is over
  if (penaltySeconds <= 0 && gameState->state == PENALISED){
    gameState->state = simulationState->state;
  }

  if (gameState->state == PENALISED && simulationState->state != PLAYING){
    gameState->state = simulationState->state;
    penaltySeconds = -dt;
  }

  if (gameState->state == PENALISED){
    setPenaltyPosition(simulationMem);
  }

  // change frame info
  frameInfo->seconds_since_start += dt;
  frameInfo->frame_id++;
  if (penaltySeconds > 0)
    penaltySeconds -= dt;
  if (kickSeconds > 0)
    kickSeconds -= dt;
  if (kickHitSeconds > 0)
    kickHitSeconds -= dt;
  if (getupSeconds > 0)
    getupSeconds -= dt;

  // check if kick is complete
  if (kickSeconds <= 0.0){
    kickRequest->kick_running_ = false;
    kickRequest->kick_type_ = Kick::NO_KICK;
    walkRequest->perform_kick_ = false;
    kickSeconds = 0;
  }

  if (kickHitSeconds <= 0)
    kickHitSeconds = 0;


  if (odometry->getting_up_side_ != Getup::NONE)
    diving = Dive::NONE;

  // check if get up is complete
  if (getupSeconds <= 0 && odometry->getting_up_side_ != Getup::NONE){
    odometry->getting_up_side_ = Getup::NONE;

    WorldObject* truthRobot = &(simulationMem->objects_[index_]);


    // with some small prob... get up the wrong direction
    float randPct = ((float)rand())/((float)RAND_MAX);
    if (randPct < 0.05){
      // pick a dir to add 90 deg
      randPct = ((float)rand())/((float)RAND_MAX);
      if (PRINT) cout << index_ << " getting up in random direction " << endl;
      if (randPct < 0.33)
        truthRobot->orientation -= M_PI/2.0;
      else if (randPct < 0.66)
        truthRobot->orientation += M_PI/2.0;
      
    // if rolled, turn 90 deg sideway upon getup
    } else if (fabs(sensors->values_[angleX]) > 0.5){
      // rolled right and front getup will face right
      // rolled left and back getup will face right
      if ((sensors->values_[angleX] > 0 && odometry->getting_up_side_ == Getup::FRONT) || (sensors->values_[angleX] < 0 && odometry->getting_up_side_ == Getup::BACK)){
        truthRobot->orientation -= M_PI/2.0;
      } else {
        truthRobot->orientation += M_PI/2.0;
      }
    }
    // if tilted, still mainly same direction

    // some angle noise upon getup
    randPct = ((float)rand())/((float)RAND_MAX)-0.5;
    truthRobot->orientation += odometryErrorFactor*0.1*M_PI*randPct;

    truthRobot->orientation = normalizeAngle(truthRobot->orientation);

    // also move the robot up to 50 cm in each dir upon getting up
    randPct = ((float)rand())/((float)RAND_MAX)-0.5;
    float randDist = randPct * 1000;
    truthRobot->loc.x += randDist;
    randPct = ((float)rand())/((float)RAND_MAX)-0.5;
    randDist = randPct * 1000;
    truthRobot->loc.y += randDist;
    

    sensors->values_[angleX] = 0;
    sensors->values_[angleY] = 0;

  }


  // update sonar observations - to see changes in walk parameters
  processedSonar->on_center_ = false;
  processedSonar->on_left_ = false;
  processedSonar->on_right_ = false;
  processedSonar->bump_left_ = false;
  processedSonar->bump_right_ = false;

  processedSonar->center_distance_ = 2550;
  processedSonar->left_distance_ = 2550;
  processedSonar->right_distance_ = 2550;

  // update current walk velocities
  walkInfo->robot_velocity_ = absWalkVel;
  walkInfo->robot_velocity_frac_ = relWalkVel;

  WorldObject* truthRobot = &(simulationMem->objects_[index_]);
 
  for (int i = WO_TEAM_FIRST; i <= WO_OPPONENT_LAST; i++) {
    // skip us
    if (i == index_) continue;
    WorldObject* wo = &(simulationMem->objects_[i]);

    float distance = truthRobot->loc.getDistanceTo(wo->loc) / 1000.0 - 0.1;
    float bearing = truthRobot->loc.getBearingTo(wo->loc, truthRobot->orientation);
    
    if (distance < 0.4 && distance > 0 && !DEBUGGING_POSITIONING) {

      // Center
      // Todd: lets say its nearly always center (since it is)
      if (bearing < M_PI / 3 && bearing > - M_PI / 3) {
        if (processedSonar->on_center_) {
          processedSonar->center_distance_ = min(processedSonar->center_distance_, distance);
        } else {
          processedSonar->center_distance_ = distance;
          processedSonar->on_center_ = true;
        }
      }
      // Right
      else if (bearing <= - M_PI / 6 && bearing > -5 * M_PI / 12) {
        if (processedSonar->on_right_) {
          processedSonar->right_distance_ = min(processedSonar->right_distance_, distance);
        } else {
          processedSonar->right_distance_ = distance;
          processedSonar->on_right_ = true;
        }
      }
      // Left
      else if (bearing >= M_PI / 6 && bearing < 5 * M_PI / 12) {
        if (processedSonar->on_left_) {
          processedSonar->left_distance_ = min(processedSonar->left_distance_, distance);
        } else {
          processedSonar->left_distance_ = distance;
          processedSonar->on_left_ = true;
        }
      }
    }

    // simulate bump sensor
    if (false && wo->distance < 350) {
      if (fabs(M_PI/2.0 - bearing) < DEG_T_RAD*30.0){
        // left bump
        processedSonar->bump_left_ = true;
      }
      if (fabs(-M_PI/2.0 - bearing) < DEG_T_RAD*30.0){
        // right bump
        processedSonar->bump_right_ = true;
      }
    }
  } // opponents

  for (int i = 0; i < 2; i++){
    jointValues->changes_[i] = jointValues->values_[i] - jointValues->prevValues_[i];
    jointValues->prevValues_[i] = jointValues->values_[i];
  }
  
  if (DEBUGGING_POSITIONING)
    core->lua_->call("strategy.DEBUGGING_POSITIONING = true;");
  else
    core->lua_->call("strategy.DEBUGGING_POSITIONING = false;");
}


void SimulatedPlayer::updateGroundTruthInputs(WorldObjectBlock* simulationMem){

  // update where we think ball, opponents, teammates are
  // from simulation memory
  worldObjects->reset();

  for (int i = 0; i <= WO_OPPONENT_LAST; i++){
    if (team_ == TEAM_BLUE){
      worldObjects->objects_[i].loc = simulationMem->objects_[i].loc;
      worldObjects->objects_[i].orientation = simulationMem->objects_[i].orientation;
      worldObjects->objects_[i].absVel = simulationMem->objects_[i].absVel;
      if (i >= WO_OPPONENT_FIRST){
        // fill in opponent mem
        opponentMem->alpha[i-WO_OPPONENT_FIRST] = 1.0;
        opponentMem->X00[i-WO_OPPONENT_FIRST] = simulationMem->objects_[i].loc.x/10.0;
        opponentMem->X10[i-WO_OPPONENT_FIRST] = simulationMem->objects_[i].loc.y/10.0;
        opponentMem->P00[i-WO_OPPONENT_FIRST] = 10.0;
        opponentMem->P11[i-WO_OPPONENT_FIRST] = 10.0;
      }
    } else {
      // have to swap players and opponent indices for red team
      if (i == 0){
        worldObjects->objects_[i].loc = -simulationMem->objects_[i].loc;
        worldObjects->objects_[i].orientation = normalizeAngle(simulationMem->objects_[i].orientation + M_PI);
        worldObjects->objects_[i].absVel = -simulationMem->objects_[i].absVel;
      } else if (i <= WO_TEAM_LAST){
        worldObjects->objects_[i+WO_TEAM_LAST].loc = -simulationMem->objects_[i].loc;
        worldObjects->objects_[i+WO_TEAM_LAST].orientation = normalizeAngle(simulationMem->objects_[i].orientation + M_PI);
        worldObjects->objects_[i+WO_TEAM_LAST].absVel = -simulationMem->objects_[i].absVel;
        // fill in opponent mem
        opponentMem->alpha[i-1] = 1.0;
        opponentMem->X00[i-1] = -simulationMem->objects_[i].loc.x/10.0;
        opponentMem->X10[i-1] = -simulationMem->objects_[i].loc.y/10.0;
        opponentMem->P00[i-1] = 10.0;
        opponentMem->P11[i-1] = 10.0;
      } else {
        worldObjects->objects_[i-WO_TEAM_LAST].loc = -simulationMem->objects_[i].loc;
        worldObjects->objects_[i-WO_TEAM_LAST].orientation = normalizeAngle(simulationMem->objects_[i].orientation + M_PI);
        worldObjects->objects_[i-WO_TEAM_LAST].absVel = -simulationMem->objects_[i].absVel;
      }
    }
  } // copy all players and ball into our memory

  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);
  WorldObject* robot = &(worldObjects->objects_[self_]);
  robot->sdOrientation = 25.0*DEG_T_RAD;


  // update ball relative velocity from absolute
  // have to make a copy first because rotate rotates the actual point
  ball->relVel = ball->absVel;
  ball->relVel.rotate(-robot->orientation);

  ball->relPos = ball->loc;
  ball->relPos = ball->relPos.globalToRelative(robot->loc, robot->orientation);

  ball->relOrientation = robot->loc.getBearingTo(ball->loc, robot->orientation);

  for (int i = 0; i < NUM_WORLD_OBJS; i++){
    if (i == WO_ROBOT_CLUSTER)
      continue;
    WorldObject* wo = &(worldObjects->objects_[i]);
    // calculate distance and bearing to each object
    wo->distance = robot->loc.getDistanceTo(worldObjects->objects_[i].loc);
    wo->bearing = robot->loc.getBearingTo(worldObjects->objects_[i].loc,
                                          robot->orientation);

    // decide if seen depending on pan
    if (fabs(jointValues->values_[HeadPan] - wo->bearing) < FOVx/2.0){
      wo->seen = true;
      wo->frameLastSeen = frameInfo->frame_id;
      float diff = jointValues->values_[HeadPan] - wo->bearing;
      wo->imageCenterX = iparams_.width/2.0 + (diff / (FOVx/2.0) * iparams_.width/2.0);
      wo->imageCenterY = iparams_.height/2.0;
      wo->visionDistance = wo->distance;
      wo->visionBearing = wo->bearing;
    }
  }

  ball->seen = true;
  ball->frameLastSeen = frameInfo->frame_id;

}


void SimulatedPlayer::updateFakeVisionInputs(WorldObjectBlock* simulationMem){
  worldObjects->reset();

  // no vision while fallen, diving or getting up
  if (odometry->getting_up_side_ != Getup::NONE || odometry->fall_direction_ != Fall::NONE || behavior->keeperDiving != Dive::NONE) 
    return;

  // true robot position
  WorldObject* truthRobot = &(simulationMem->objects_[index_]);

  // dont see goals if headpan moved more than 2.5 deg
  // dont see other objects if headpan moved more than 4.0 deg
  float panDiff = jointValues->changes_[HeadPan];
  if (fabs(panDiff) > 0.5 * DEG_T_RAD){
    panMoving = true;
  } else {
    if (panMoving)
      panStopTime = frameInfo->seconds_since_start;
    panMoving = false;
  }
  if (fabs(panDiff) > 4.0 * DEG_T_RAD) return;

  // ball
  WorldObject* truthBall = &(simulationMem->objects_[WO_BALL]);
  float bearing = truthRobot->loc.getBearingTo(truthBall->loc,truthRobot->orientation);
  float distance = truthRobot->loc.getDistanceTo(truthBall->loc);

  WorldObject* obsBall = &(worldObjects->objects_[WO_BALL]);

  // in FOV
  if (fabs(jointValues->values_[HeadPan] - bearing) < FOVx/2.0){
    float missedObsRate = 1.0/15.0;
    if (distance > 2000)
      missedObsRate = 1.0/8.0;
    else if (distance > 3500)
      missedObsRate = 1999.0/2000.0;
    float randPct = ((float)rand())/((float)RAND_MAX);
    bool visible = true;
    // check for other robots obstructing us
    for (int j = 1; j < WO_OPPONENT_LAST; j++){
      if (index_ == j) continue;
      WorldObject* truthOther = &(simulationMem->objects_[j]);
      float otherBearing = truthRobot->loc.getBearingTo(truthOther->loc,truthRobot->orientation);
      float otherDistance = truthRobot->loc.getDistanceTo(truthOther->loc);
      if (otherDistance < distance && fabs(normalizeAngle(otherBearing - bearing)) < 4.0*DEG_T_RAD){
        visible = false;
        //cout << "View of ball obstructed from " << index_ << " by " << j << endl;
        break;
      }
    }
    if (visible && randPct > (missedObsRate *missedObsFactor)){
      // seen
      obsBall->seen = true;
      obsBall->visionConfidence = 1.0;
      obsBall->frameLastSeen = frameInfo->frame_id;
      float diff = jointValues->values_[HeadPan] - bearing;
      obsBall->imageCenterX = iparams_.width/2.0 + (diff / (FOVx/2.0) * iparams_.width/2.0);
      obsBall->imageCenterY = iparams_.height/2.0;
      // add distance and bearing noise
      float randNoise = ((float)rand())/((float)RAND_MAX)-0.5;
      obsBall->visionDistance = distance + randNoise * visionErrorFactor * 0.05*distance;// up to 5% distance error
      obsBall->visionBearing = bearing + randNoise * visionErrorFactor * 5.0*DEG_T_RAD;// up to 5 deg bearing error
    }
  }

  // opponents
  int firstOpp = 1;
  if (team_ == TEAM_BLUE) firstOpp = 5;
  int oppSeen = 0;
  for (int i = 0; i < 4; i++){
    WorldObject* truthWO = &(simulationMem->objects_[i+firstOpp]);
    float bearing = truthRobot->loc.getBearingTo(truthWO->loc,truthRobot->orientation);
    float distance = truthRobot->loc.getDistanceTo(truthWO->loc);

    WorldObject* obsWO = &(worldObjects->objects_[5+oppSeen]);
    // in FOV
    if (fabs(jointValues->values_[HeadPan] - bearing) < FOVx/2.0){
      float missedObsRate = 1.0/5.0;
      float randPct = ((float)rand())/((float)RAND_MAX);
      // only see opponents between 0.5 and 2.5 meters
      if (randPct > (missedObsRate *missedObsFactor) && distance > 500 && distance < 2500){
        // seen
        obsWO->seen = true;
        obsWO->visionConfidence = 1.0;
        oppSeen++;
        float diff = jointValues->values_[HeadPan] - bearing;
        obsWO->imageCenterX = iparams_.width/2.0 + (diff / (FOVx/2.0) * iparams_.width/2.0);
        obsWO->imageCenterY = iparams_.height/2.0;
        // add distance and bearing noise
        float randNoise = ((float)rand())/((float)RAND_MAX)-0.5;
        obsWO->visionDistance = distance + randNoise * visionErrorFactor * 0.25*distance;// up to 25% distance error
        obsWO->visionBearing = bearing + randNoise * visionErrorFactor * 7.0*DEG_T_RAD;// up to 7 deg bearing error

      }
    }
  } // opp loop


  // center circle
  WorldObject* truthWO = &(simulationMem->objects_[WO_CENTER_CIRCLE]);
  bearing = truthRobot->loc.getBearingTo(truthWO->loc,truthRobot->orientation);
  distance = truthRobot->loc.getDistanceTo(truthWO->loc);

  WorldObject* obsWO = &(worldObjects->objects_[WO_CENTER_CIRCLE]);
  // in FOV
  if (fabs(jointValues->values_[HeadPan] - bearing) < FOVx/2.0){
    float missedObsRate = 1.0/5.0;
    float randPct = ((float)rand())/((float)RAND_MAX);
    // see circle up to 3 m
    if (randPct > (missedObsRate *missedObsFactor) && distance < 3000){
      // seen
      obsWO->seen = true;
      oppSeen++;
      float diff = jointValues->values_[HeadPan] - bearing;
      obsWO->imageCenterX = iparams_.width/2.0 + (diff / (FOVx/2.0) * iparams_.width/2.0);
      obsWO->imageCenterY = iparams_.height/2.0;
      // add distance and bearing noise
      float randNoise = ((float)rand())/((float)RAND_MAX)-0.5;
      obsWO->visionDistance = distance + randNoise * visionErrorFactor * 0.15*distance;// up to 15% distance error
      obsWO->visionBearing = bearing + randNoise * visionErrorFactor * 5.0*DEG_T_RAD;// up to 5 deg bearing error
      obsWO->visionConfidence = 1.0;


    }
  }

  // lines
  int seenLines = 0;
  Point2D* linePts = new Point2D[4];

  for (int i = WO_OPP_GOAL_LINE; i <= WO_BOTTOM_SIDE_LINE; i++){
    if (seenLines >= 4) break;
    WorldObject* truthWO = &(simulationMem->objects_[i]);
    
    // see if either end point, center point, or closest point are visible
    linePts[0] = truthWO->lineLoc.getPointOnLineClosestTo(truthRobot->loc);
    linePts[1] = truthWO->lineLoc.center;
    linePts[2] = truthWO->loc;
    linePts[3] = truthWO->endLoc;

    WorldObject* obsWO = &(worldObjects->objects_[WO_UNKNOWN_FIELD_LINE_1+seenLines]);
    bool pointsVisible = false;

    int j;
    for (j = 0; j < 4; j++){
      bearing = truthRobot->loc.getBearingTo(linePts[j],truthRobot->orientation);
      distance = truthRobot->loc.getDistanceTo(linePts[j]);
      // make sure point is on line
      if (i >= 48 && i <= 51 && j == 0 && fabs(linePts[j].x) < 2400) continue;
      if ((i == 43 || i == 45) && j == 0 && fabs(linePts[j].y) > 1100) continue;
      if (j == 0 && fabs(linePts[j].x) > FIELD_X / 2.0) continue;
      if (j == 0 && fabs(linePts[j].y) > FIELD_Y / 2.0) continue;
      // in FOV
      if (fabs(jointValues->values_[HeadPan] - bearing) < FOVx/2.0){
        pointsVisible = true;
        break;
      }
    }

    if (pointsVisible){
      float missedObsRate = 1.0/3.0;
      float randPct = ((float)rand())/((float)RAND_MAX);
      // see lines up to 2.5 m
      if (randPct > (missedObsRate *missedObsFactor) && distance < 2500 && distance > 5 && truthRobot->loc.getDistanceTo(linePts[0]) > 5){
        // seen
        obsWO->seen = true;
        seenLines++;
        float diff = jointValues->values_[HeadPan] - bearing;
        obsWO->imageCenterX = iparams_.width/2.0 + (diff / (FOVx/2.0) * iparams_.width/2.0);
        obsWO->imageCenterY = iparams_.height/2.0;
        // add distance and bearing noise
        float randNoise = ((float)rand())/((float)RAND_MAX)-0.5;

        float newBearing = truthRobot->loc.getBearingTo(linePts[0],truthRobot->orientation);
        float newDistance = truthRobot->loc.getDistanceTo(linePts[0]);

        float distanceError = randNoise * visionErrorFactor * 0.15*newDistance;// up to 15% distance error
        obsWO->visionDistance = newDistance + distanceError;
        float bearingError = randNoise * visionErrorFactor * 8.0*DEG_T_RAD;// up to 8 deg bearing error
        obsWO->visionBearing = newBearing + bearingError;
        obsWO->visionConfidence = 1.0;
        
        Point2D relPt(distance+distanceError, bearing, POLAR);
        obsWO->visionPt1 = relPt + Point2D(400, obsWO->visionBearing+M_PI/2.0, POLAR);
        obsWO->visionPt2 = relPt + Point2D(400, obsWO->visionBearing-M_PI/2.0, POLAR);

        //cout << index_ << " see line " << i << " at point " << j << ": " << linePts[j] << " at dist: " << distance << " and bearing: " << bearing*RAD_T_DEG << " closestPt: " << linePts[0] << " vision distance: " << obsWO->visionDistance << ", bearing: " << RAD_T_DEG * obsWO->visionBearing << endl;

      } // line was seen with random prob
    } // line is visible
  } // line loop
      


  // must have stoped panning for x seconds
  if (panMoving || (frameInfo->seconds_since_start - panStopTime) < 0.1){
    //cout << index_ << " no goal vision because panMoving: " << panMoving << " at " << panDiff*RAD_T_DEG << " or only stopped for " << (frameInfo->seconds_since_start - panStopTime) << endl;
    return;
  } 
  

  // goal posts
  int seenPostCount = 0;
  int firstPost = 0;
  for (int i = WO_OWN_LEFT_GOALPOST; i <= WO_OPP_RIGHT_GOALPOST; i++){
    WorldObject* truthWO = &(simulationMem->objects_[i]);
    float bearing = truthRobot->loc.getBearingTo(truthWO->loc,truthRobot->orientation);
    float distance = truthRobot->loc.getDistanceTo(truthWO->loc);
    WorldObject* obsWO = &(worldObjects->objects_[i]);

    // in FOV
    if (fabs(jointValues->values_[HeadPan] - bearing) < FOVx/2.0){
      float missedObsRate = 1.0/10.0;
      if (distance > 3000) 
        missedObsRate = 1.0/3.0;
      float randPct = ((float)rand())/((float)RAND_MAX);
      if (randPct > (missedObsRate *missedObsFactor) && distance < 7000){
        // seen
        obsWO->seen = true;
        if (seenPostCount == 0) firstPost = i;
        seenPostCount++;
        float diff = jointValues->values_[HeadPan] - bearing;
        obsWO->imageCenterX = iparams_.width/2.0 + (diff / (FOVx/2.0) * iparams_.width/2.0);
        obsWO->imageCenterY = iparams_.height/2.0;
        // add distance and bearing noise
        float randNoise = ((float)rand())/((float)RAND_MAX)-0.5;
        obsWO->visionDistance = distance + randNoise * visionErrorFactor * 0.25*distance;// up to 25% distance error
        obsWO->visionBearing = bearing + randNoise * visionErrorFactor * 5.0*DEG_T_RAD;// up to 5 deg bearing error
        obsWO->visionConfidence = 1.0;


      }
    }
  }

  // actually have to fill those into unknown post spot
  if (seenPostCount == 1){
    // fill in unknown post
    WorldObject* obsWO = &(worldObjects->objects_[WO_UNKNOWN_GOALPOST]);
    WorldObject* known = &(worldObjects->objects_[firstPost]);
    obsWO->seen = true;
    obsWO->frameLastSeen = frameInfo->frame_id;
    obsWO->imageCenterX = known->imageCenterX;
    obsWO->imageCenterY = known->imageCenterY;
    obsWO->visionDistance = known->visionDistance;
    obsWO->visionBearing = known->visionBearing;
    obsWO->visionConfidence = 1.0;
    known->seen = false;

  }
  else if (seenPostCount == 2){
    // fill in left and right post and goal with average of two
    float sumX = 0;
    float sumDist = 0;
    float sumBear = 0;
    for (int i = 0; i < 2; i++){
      WorldObject* obsWO = &(worldObjects->objects_[WO_UNKNOWN_LEFT_GOALPOST+i]);
      WorldObject* known = &(worldObjects->objects_[firstPost+(i*2)]);
      obsWO->seen = true;
      obsWO->frameLastSeen = frameInfo->frame_id;
      obsWO->imageCenterX = known->imageCenterX;
      obsWO->imageCenterY = known->imageCenterY;
      obsWO->visionDistance = known->visionDistance;
      obsWO->visionBearing = known->visionBearing;
      obsWO->visionConfidence = 1.0;

      known->seen = false;
      sumX += obsWO->imageCenterX;
      sumDist += obsWO->visionDistance;
      sumBear += obsWO->visionBearing;

    }
    WorldObject* obsWO = &(worldObjects->objects_[WO_UNKNOWN_GOAL]);
    obsWO->seen = true;
    obsWO->frameLastSeen = frameInfo->frame_id;
    obsWO->imageCenterX = sumX / 2.0;
    obsWO->imageCenterY = iparams_.height/2.0;
    obsWO->visionDistance = sumDist / 2.0;
    obsWO->visionBearing = sumBear / 2.0;
    obsWO->visionConfidence = 1.0;

  } else if (seenPostCount > 2){
    //cout << index_ << " error saw more than 2 posts: " << seenPostCount << endl;
  }
}


bool SimulatedPlayer::updateOutputs(WorldObjectBlock* simulationMem){

  // our location
  WorldObject* robot = &(worldObjects->objects_[self_]);
  WorldObject* ball = &(worldObjects->objects_[WO_BALL]);

  // update walk info so walk vel ramp-ups work properly
  walkInfo->robot_velocity_ = Pose2D(0,0,0);
  walkInfo->robot_velocity_frac_ = Pose2D(0,0,0);
  walkInfo->walk_is_active_ = false;
  odometry->displacement = Pose2D(0,0,0);
  odometry->didKick = false;
  odometry->standing = true;

  // possibly reset our walk to target
  if (walkRequest->motion_ != WalkRequestBlock::WALK && walkRequest->motion_ != WalkRequestBlock::WAIT && !walkRequest->walk_to_target_){
    walkToTarget = false;
  }

  // if we're walking to target and got there... stop
  if (walkToTarget && (target.getDistanceTo(robot->loc) < 20)){
    //cout << "close to target, error: " << target.getDistanceTo(robot->loc) << endl;
    walkToTarget = false;
  }

  // update our location and ball location based on walk/kick requests
  if (walkRequest->motion_ == WalkRequestBlock::WALK || walkToTarget){
    // figure out max speed from al walk param block

    float rotateVel = 0;
    float fwdVel = 0;
    float leftVel = 0;

    // walk to target walk
    if (walkRequest->walk_to_target_){
      Point2D relTarget = behavior->absTargetPt.globalToRelative(robot->loc, robot->orientation);
      //relTarget.x -= 20.0;
      target = relTarget.relativeToGlobal(robot->loc, robot->orientation);
      walkToTarget = true;
      //std::cout << "walk to target in walk_request" << std::endl;

    }
    if (walkToTarget){
      // multiply these by the appropriate factor
      Point2D relTarget = target.globalToRelative(robot->loc, robot->orientation);
      float targetX = relTarget.x;
      float targetY = relTarget.y;

      //cout << index_ << " Walk to target, " << walkToTarget << ", : " << walkRequest->walk_to_target_ << ", pt: " << targetX << ", " << targetY << ", dist: " << target.getDistanceTo(robot->loc) << endl;


      // use behavior target pt
      rotateVel = 0;
      if (targetX > maxFwdVel) {
        fwdVel = maxFwdVel;
      } else {
        // do a minimum fwd vel for this
        //if (targetX > 0 && targetX < 160){
        //  fwdVel = 160;
        //} else {
          fwdVel = targetX;
          //}
      }
      if (fabs(targetY) > maxSideVel*2.0){
        if (targetY > 0)
          leftVel = maxSideVel;
        else
          leftVel = -maxSideVel;
      } else {
        leftVel = targetY;
      }
      //cout << "walk to point " << walkRequest->target_point_ << endl;
      //cout << "walk to point vels: rot: " << rotateVel << " fwd: " << fwdVel << " left: " << leftVel << endl;
    }
    // pct speed walk
    else if (walkRequest->percentage_speed_){
      if (walkRequest->speed_.rotation > 0)
        rotateVel = maxTurnVel * walkRequest->speed_.rotation;
      else
        rotateVel = maxTurnVel * walkRequest->speed_.rotation;
      fwdVel = maxFwdVel * walkRequest->speed_.translation.x;
      leftVel = maxSideVel * walkRequest->speed_.translation.y;
      //cout << "pct vel: rot: " << walkRequest->speed_.rotation << " fwd: " << walkRequest->speed_.translation.x << " left: " << walkRequest->speed_.translation.y << endl;
    }
    // abs speed walk
    else {
      // TODO: do we even do this at all?
      cout << "ERROR: simulation does not handle absolute speed requests" << endl;
      /*
        rotateVel = walkRequest->speed_.rotation;
        fwdVel = walkRequest->speed_.translation.x;
        leftVel = walkRequest->speed_.translation.y;
        rotateVel = crop(rotateVel, -maxTurnVel, maxTurnVel);
        fwdVel = crop(fwdVel, -maxFwdVel, maxFwdVel);
        leftVel = crop(leftVel, -maxSideVel, maxSideVel);
        //cout << "absolute velocities rot: " << walkRequest->speed_.rotation << " fwd: " << walkRequest->speed_.translation.x << " left: " << walkRequest->speed_.translation.y << endl;
        */
    }



    // update walk info so walk vel ramp-ups work properly
    walkInfo->robot_velocity_ = Pose2D(rotateVel, fwdVel, leftVel);
    walkInfo->robot_velocity_frac_ = Pose2D(rotateVel / maxTurnVel, fwdVel / maxFwdVel, leftVel / maxSideVel);
    walkInfo->walk_is_active_ = true;
    odometry->standing = false;

    //cout << "updating with speeds rotate: " << rotateVel << ", Fwd: " << fwdVel << ", Side: " << leftVel << endl;

    // lets move the robot for 0.1 seconds at these speeds
    Point2D oneStepMove = Point2D(fwdVel, leftVel) * dt;
    float oneStepRotate = rotateVel * dt;

    // update to simulation mem
    WorldObject* truthRobot = &(simulationMem->objects_[index_]);

    odometry->displacement = Pose2D(oneStepRotate, oneStepMove.x, oneStepMove.y);

    // if walking .... fall over about once every 3 minutes
    float randPct = ((float)rand())/((float)RAND_MAX);

    // more often in center (for testing purposes)
    // 1/5400 is 1/3min, 1/3600 is 1/2min, 1/1800 is 1/min
    if (!DEBUGGING_POSITIONING && (randPct < (1.0 / 3600.0)))// || (truthRobot->loc.getDistanceTo(Point2D(0,0)) < 800 && randPct < (1.0 / 1000.0)))
      setFallen();
  

    // add walk noise
    // up to 20% error on x vel
    randPct = ((float)rand())/((float)RAND_MAX)-0.5;
    oneStepMove.x += randPct*odometryErrorFactor*0.4*oneStepMove.x;

    // up to 20% error on y vel
    randPct = ((float)rand())/((float)RAND_MAX)-0.5;
    oneStepMove.y += randPct*odometryErrorFactor*0.4*oneStepMove.y;

    // up to 20% error + 1 degree error on rotate vel
    randPct = ((float)rand())/((float)RAND_MAX)-0.5;
    oneStepRotate += randPct*odometryErrorFactor*0.4*oneStepRotate;
    randPct = ((float)rand())/((float)RAND_MAX)-0.5;
    oneStepRotate += randPct*odometryErrorFactor*M_PI/180.0;


    Point2D newPos =
      oneStepMove.relativeToGlobal( truthRobot->loc, truthRobot->orientation);
    truthRobot->loc = newPos;
    truthRobot->orientation = normalizeAngle(truthRobot->orientation + oneStepRotate);

  }
  absWalkVel = walkInfo->robot_velocity_;
  relWalkVel = walkInfo->robot_velocity_frac_;

  // start timer
  if ((kickRequest->kick_type_ != Kick::NO_KICK || walkRequest->perform_kick_) && kickSeconds <= 0.0){
    float factor = 1;
    // walk kicks are faster
    if (walkRequest->perform_kick_){
      factor = 0.4;
    }
    //cout << index_ << " kick time: " << factor << " type: " << kickRequest->kick_type_ << " wr: " << walkRequest->perform_kick_ << " choice: " << behavior->kickChoice<< endl;
    kickSeconds = kickFullTime * factor;
    kickHitSeconds = kickHitTime * factor;
    kickRequest->kick_running_ = true;
  }

  // kick abort
  if (kickRequest->kick_type_ == Kick::ABORT){
    kickRequest->kick_type_ = Kick::NO_KICK;
    kickSeconds = 0;
    kickHitSeconds = 0;
    kickRequest->kick_running_ = false;
  }

  // kick gets executed kickHitTime seconds in
  if ((kickRequest->kick_type_ != Kick::NO_KICK || walkRequest->perform_kick_) && kickRequest->kick_type_ != Kick::ABORT && kickHitSeconds <= 0.1 && kickHitSeconds > 0){

    //cout << index_ << " attempt kick type " << kickRequest->kick_type_ << " with dist: " << kickRequest->desired_distance_ << " and angle " << kickRequest->desired_angle_ << " walk req: " << walkRequest->perform_kick_ << ", dist " << walkRequest->kick_distance_ << " head: " << walkRequest->kick_heading_ << endl;
    kickHitSeconds = 0;
    
    // this stuff only happens if ball is at robot!
    if (ball->relPos.x < 210 && fabs(ball->relPos.y) < 90 && ball->relPos.x > 0){

      // assume ball velocity at heading
      float vel = kickRequest->desired_distance_;
      float heading = kickRequest->desired_angle_;

      // unless this was from walk
      if (walkRequest->perform_kick_){
        vel = walkRequest->kick_distance_;
        heading = walkRequest->kick_heading_;
      }

      odometry->didKick = true;
      odometry->kickHeading = heading;
      odometry->kickVelocity = vel / 1.2;

      // add up to 10 degree error
      float randPct = ((float)rand())/((float)RAND_MAX)-0.5;
      heading += kickErrorFactor*randPct*20.0*DEG_T_RAD;
      // add up to 40 % vel error
      float randPct2 = ((float)rand())/((float)RAND_MAX)-0.5;
      vel += randPct2*kickErrorFactor*0.8*vel;

      // never longer than 8000
      if (vel > 8000) vel = 8000;

      // never shorter than 10
      if (vel < 10) vel = 10;

      //      cout << team_ << " desired vel: " << kickRequest->desired_distance_ << " actual: " << vel << ", desired heading: " << RAD_T_DEG*kickRequest->desired_angle_ << " actual: " << heading*RAD_T_DEG << endl;

      kickRequest->kick_type_ = Kick::NO_KICK;

      // update to simulation mem
      WorldObject* truthRobot = &(simulationMem->objects_[index_]);
      WorldObject* truthBall = &(simulationMem->objects_[WO_BALL]);

      Point2D absVel(vel, truthRobot->orientation + heading, POLAR);
      if (!DEBUGGING_POSITIONING) {
        truthBall->absVel = absVel;
        return true;
      }
    } else {
      //cout << " kick failed, ball dist: " << ball->distance << " rel: " << ball->relPos << endl;
      kickRequest->kick_type_ = Kick::NO_KICK;

    }
  }

  // kick or getup makes head to go to 0
  if (kickRequest->kick_running_ || odometry->getting_up_side_ != Getup::NONE || odometry->fall_direction_ != Fall::NONE){
    jointCommands->head_yaw_angle_change_ = false;
    jointCommands->angles_[HeadYaw] = 0;
    jointCommands->send_head_yaw_angle_ = true;
    jointCommands->head_yaw_angle_time_ = 200;
  }

  if (diving != behavior->keeperDiving && behavior->keeperDiving != Dive::NONE)
    if (PRINT) cout << index_ << " diving" << endl;


  // diving
  if (behavior->keeperDiving == Dive::RIGHT || (diving == Dive::RIGHT && sensors->values_[angleX] < M_PI/2.0)){
    diving = Dive::RIGHT;
    if (sensors->values_[angleX] < M_PI/2.0){
      sensors->values_[angleX] += M_PI/20.0;
    }
  } else if (behavior->keeperDiving == Dive::LEFT || (diving == Dive::LEFT && sensors->values_[angleX] > -M_PI/2.0)){
    diving = Dive::LEFT;
    if (sensors->values_[angleX] > -M_PI/2.0){
      sensors->values_[angleX] -= M_PI/20.0;
    }
  } else if (behavior->keeperDiving == Dive::NONE){
    diving = Dive::NONE;
  }

  // start getting up
  if (walkRequest->motion_ == WalkRequestBlock::FALLING && odometry->getting_up_side_ == Getup::NONE){
    if (getupSeconds <= 0) getupSeconds = getupTimeLength;
    behavior->keeperDiving = Dive::NONE;
    odometry->getting_up_side_ = Getup::UNKNOWN;
  }

  // continue get up, finish cross and choose a side to get up from
  if (getupSeconds < (getupTimeLength-0.5) && getupSeconds > (getupTimeLength-1.0)){
    if (sensors->values_[angleY] > 0)
      odometry->getting_up_side_ = Getup::FRONT;
    else
      odometry->getting_up_side_ = Getup::BACK;
  }

  // based on joint commands for head, update joint values
  if (jointCommands->send_head_pitch_angle_ && jointCommands->head_pitch_angle_time_ > 0){
    if (jointCommands->head_pitch_angle_change_)
      jointCommands->angles_[HeadPitch] += jointValues->values_[HeadPitch];
    float moveFrac = 1000.0* dt / jointCommands->head_pitch_angle_time_;
    float moveDist = jointCommands->angles_[HeadPitch] - jointValues->values_[HeadPitch];
    float singleMove = moveFrac*moveDist;
    jointValues->values_[HeadPitch] += singleMove;
  }

  // update head pan angle
  if (jointCommands->send_head_yaw_angle_ && jointCommands->head_yaw_angle_time_ > 0){
    if(jointCommands->head_yaw_angle_change_)
      jointCommands->angles_[HeadYaw] += jointValues->values_[HeadYaw];
    float moveFrac = 1000.0* dt / jointCommands->head_yaw_angle_time_;
    jointCommands->head_yaw_angle_time_ -= 1000.0* dt;
    float moveDist = jointCommands->angles_[HeadYaw] - jointValues->values_[HeadYaw];
    float singleMove = moveFrac*moveDist;
    jointValues->values_[HeadYaw] += singleMove;
    if (jointValues->values_[HeadYaw] > 120.0*DEG_T_RAD)
      jointValues->values_[HeadYaw] = 120.0*DEG_T_RAD;
    else if (jointValues->values_[HeadYaw] < -120.0*DEG_T_RAD)
      jointValues->values_[HeadYaw] = -120.0*DEG_T_RAD;
  }

  return false;
}


std::vector<std::string> SimulatedPlayer::getTextDebug(){
  return core->textlog_.textEntries;
}

void SimulatedPlayer::changeState(int state){
  gameState->state = state;
}

void SimulatedPlayer::restartLua(){
  core->lua_->initLua();
}

void SimulatedPlayer::setStrategy(){
  core->lua_->doStrategyCalculations();
}

float SimulatedPlayer::crop(float val, float min, float max){
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

void SimulatedPlayer::setPenalty(WorldObjectBlock* simulationMem){
  if (gameState->state == PLAYING || gameState->state == READY){
    setPenaltyPosition(simulationMem);
    // robot gets stood up on penalty
    resetCounters();
    penaltySeconds = 30.0;
    gameState->state = PENALISED;
    if (PRINT) cout << index_ << " is penalised" << endl;
  }
}

void SimulatedPlayer::resetCounters(){
  penaltySeconds = -0.1;
  kickSeconds = -0.1;
  kickHitSeconds = -0.1;
  getupSeconds = -0.1;
  odometry->getting_up_side_ = Getup::NONE;
  diving = Dive::NONE;
  walkToTarget = false;
  sensors->values_[angleX] = 0;
  sensors->values_[angleY] = 0;
}

void SimulatedPlayer::setPenaltyPosition(WorldObjectBlock* simulationMem){
  // affect true location in simulation mem

  WorldObject* robot = &(simulationMem->objects_[index_]);
  WorldObject* ball = &(simulationMem->objects_[WO_BALL]);
  if (ball->loc.y > 0){
    robot->loc.y = -FIELD_Y/2.0 - 100.0;
    robot->orientation = M_PI/2.0;
  } else {
    robot->loc.y = FIELD_Y/2.0 + 100.0;
    robot->orientation = -M_PI/2.0;
  }
  // check that no one is already at x
  float penX = -1200;
  if (team_ == TEAM_RED) penX = 1200;
  for (int i = 0; i < 4; i++){
    float desiredX = penX + i * 380.0;
    bool posTaken = false;
    for (int j = WO_TEAM1; j <= WO_OPPONENT4; j++){
      if (j == index_) continue;
      if (simulationMem->objects_[j].loc.getDistanceTo(Point2D(desiredX,robot->loc.y)) < 100){
        posTaken = true;
        break;
      }
    }
    if (!posTaken){
      robot->loc.x = desiredX;
      break;
    }
    desiredX = penX + -i * 380.0;
    posTaken = false;
    for (int j = WO_TEAM1; j <= WO_OPPONENT4; j++){
      if (j == index_) continue;
      if (simulationMem->objects_[j].loc.getDistanceTo(Point2D(desiredX,robot->loc.y)) < 100){
        posTaken = true;
        break;
      }
    }
    if (!posTaken){
      robot->loc.x = desiredX;
      break;
    }
  }
}



void SimulatedPlayer::setFallen(){
  if (PRINT) cout << index_ << " has fallen " << endl;
  // random fall direction
  float randPct = ((float)rand())/((float)RAND_MAX);
  // give a little extra tilt and roll
  if (randPct > 0.5){
    sensors->values_[angleY] = 0.05;
    sensors->values_[angleX] = 0.05;
  } else {
    sensors->values_[angleY] = -0.05;
    sensors->values_[angleX] = -0.05;
  } 
  randPct = ((float)rand())/((float)RAND_MAX);
  if (randPct < 0.4) // tilt fwd
    sensors->values_[angleY] = M_PI/2.0;
  else if (randPct < 0.8) // tilt back
    sensors->values_[angleY] = -M_PI/2.0;
  else if (randPct < 0.9) // tilt left
    sensors->values_[angleX] = -M_PI/2.0;
  else // tilt right
    sensors->values_[angleX] = M_PI/2.0;
}
