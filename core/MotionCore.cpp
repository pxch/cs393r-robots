#include "MotionCore.h"

#include <memory/BodyModelBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/SensorBlock.h>
#include <memory/KickParamBlock.h>
#include <memory/WalkParamBlock.h>
#include <memory/ALWalkParamBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/RobotInfoBlock.h>

#include <memory/WorldObjectBlock.h>

#include <memory/WalkRequestBlock.h>
#include <memory/ProcessedSonarBlock.h>

#include <kinematics/KinematicsModule.h>
#include <motion/KickModule.h>
#include <motion/MotionModule.h>
#include <motion/GetupModule.h>
#include <motion/WalkModule.h>
//#include <motion/HTWKWalkModule.h>
#include <motion/bhwalk/BHWalkModule.h>
#include <motion/SpecialMotionModule.h>
#include <motion/KickSideMiddleModule.h>
#include <sensor/SensorModule.h>
#include <sonar/SonarModule.h>

#include <common/InterfaceInfo.h>
#include <math/Geometry.h>

#include <common/Calibration.h>
#include <common/PIDController.h>

#include <boost/interprocess/sync/named_mutex.hpp>

MotionCore *MotionCore::inst_ = NULL;

MotionCore::MotionCore (CoreType type, bool use_shared_memory,int team_num, int player_num):
  memory_(use_shared_memory,MemoryOwner::MOTION,team_num,player_num,false),
  type_(type),
  last_frame_processed_(0),
  use_com_kick_(true),
  kinematics_(NULL),
  kick_(NULL),
  motion_(NULL),
  sensor_(NULL),
  sonar_(NULL),
  getup_(NULL),
  specialM_(NULL),
  kickside_(NULL),
  walk_(NULL),
  //htwk_walk_(NULL),
  bh_walk_(NULL),
  textlog_(NULL),
  sync_joint_angles_(NULL),
  sync_kick_request_(NULL),
  sync_odometry_(NULL),
  sync_sensors_(NULL),
  sync_walk_request_(NULL),
  sync_joint_commands_(NULL),
  sync_processed_sonar_(NULL),
  sync_kick_params_(NULL),
  sync_walk_param_(NULL),
  sync_al_walk_param_(NULL),
  sync_walk_info_(NULL),
  fps_frames_processed_(0)
{
  init();

  /*
  memory_.setBlockLogging("frame_info",true);
  memory_.setBlockLogging("graphable",true);
  memory_.setBlockLogging("walk_engine",true);
  memory_.setBlockLogging("body_model",true);
  memory_.setBlockLogging("joint_angles",true);
  memory_.setBlockLogging("processed_sensors",true);
  memory_.setBlockLogging("processed_joint_commands",true);
  memory_.setBlockLogging("processed_joint_angles",true);
  memory_.setBlockLogging("sensor_calibration",true);
  memory_.setBlockLogging("walk_param",true);
  memory_.setBlockLogging("walk_request",true);
  memory_.setBlockLogging("kick_request",true);
  memory_.setBlockLogging("kick_engine",true);

  //enableLogging("/home/todd/Nao/nao/trunk/logs/motion.log");
  //enableLogging("/home/sam/Nao/trunk/logs/motion.log");
  //enableLogging("/home/nao/logs/motion.log");
  */
}

MotionCore::~MotionCore() {
  // clean up the modules
  if (kinematics_ != NULL)
    delete kinematics_;
  if (kick_ != NULL)
    delete kick_;
  if (motion_ != NULL)
    delete motion_;
  if (sensor_ != NULL)
    delete sensor_;
  if (sonar_ != NULL)
    delete sonar_;
  if (walk_ != NULL)
    delete walk_;
  //if (htwk_walk_ != NULL)
    //delete htwk_walk_;
  if (bh_walk_ != NULL)
    delete bh_walk_;
}

void MotionCore::init() {
  if (type_ == CORE_INIT) {
    FrameInfoBlock *frame_info;
    memory_.getBlockByName(frame_info,"frame_info");
    if (frame_info->source == MEMORY_SIM) {
      std::cout << "MOTION CORE: SIM" << std::endl;
      type_ = CORE_SIM;
    } else if (frame_info->source == MEMORY_ROBOT) {
      std::cout << "MOTION CORE: ROBOT" << std::endl;
      type_ = CORE_ROBOT;
    } else {
      std::cerr << "Unknown memory type when init vision core" << std::endl;
      exit(1);
    }
  }

  inst_ = this;

  setMemoryVariables();

  initMemory();
  initModules();

  fps_time_ = frame_info_->seconds_since_start;
  time_motion_started_ = frame_info_->seconds_since_start;
}

bool MotionCore::alreadyProcessedFrame() {
  if (frame_info_->frame_id <=last_frame_processed_) {
    return true;
  } else {
    return false;
  }
}

void MotionCore::processMotionFrame() {
  unsigned int &frame_id = frame_info_->frame_id;
  if (alreadyProcessedFrame()) {
    //std::cout << "processMotionFrame, skipping frame " << frame_info_->frame_id << std::endl;
    return;
  }

  if (frame_id > last_frame_processed_ + 1) {
    std::cout << "Skipped a frame: went from " << last_frame_processed_ << " to " << frame_id << std::endl;
  }
  // frame rate
  double time_passed = frame_info_->seconds_since_start - fps_time_;
  fps_frames_processed_++;
  if (time_passed >= 10.0) {
    std::cout << "MOTION FRAME RATE: " << fps_frames_processed_ / time_passed << std::endl;
    fps_frames_processed_ = 0;
    fps_time_ = frame_info_->seconds_since_start;
  }

  // actually do stuff
  processSensorUpdate();
  // check if the get up wants us to stand
  if (getup_->needsStand()) {
    walk_request_->new_command_ = true;
    walk_request_->slow_stand_ = true;
  }


  if (bh_walk_ != NULL)
    bh_walk_->handleStepIntoKick();

  // kicks need to be before walk, so that they can change the walk request
  if (use_com_kick_)
    kick_->processFrame();
  else
    motion_->processFrame();


  if (walk_ != NULL)
    walk_->processFrame();
  //if (htwk_walk_ != NULL)
    //htwk_walk_->processFrame();
  if (bh_walk_ != NULL)
    bh_walk_->processFrame();

    //   if(kickside_->specialmotion_state_==6)
            //htwk_walk_->processFrame();
   //    else
   //    {
   //          std::cout<<"process kick test"<<std::endl;
   //          kickside_->initSideKick(2,true);
   //          kickside_->processSideKick();
   //    }

  updateOdometry();

  // override commands with getup if necessary
 //  cout<<(walk_request_->motion_==WalkRequestBlock::FALLING)<<"("<< walk_request_->fallen_counter_<<std::endl;
//  cout<<(walk_request_->motion_==WalkRequestBlock::FALLING)<<"("<<processed_sensors->values_[angleY]<<","<<processed_sensors->values_[angleX]<<")"<<std::endl;
  if (getup_->isGettingUp() || walk_request_->motion_==WalkRequestBlock::FALLING){
    // possibly init get up
    if (!getup_->isGettingUp())
      getup_->initGetup();
    getup_->processFrame();
  }

  //std::cout << "left " << body_model_->rel_parts_no_rotations_[BodyPart::left_hand].translation << " | right " << body_model_->rel_parts_no_rotations_[BodyPart::right_hand].translation << std::endl;


  last_frame_processed_ = frame_id;
}

void MotionCore::processSensorUpdate() {
  sensor_->processSensors();
  kinematics_->calculatePose();
  sonar_->processFrame();
}

void MotionCore::initModules() {
  kinematics_ = new KinematicsModule();
  kinematics_->init(&memory_,&textlog_);

  kick_ = new KickModule();
  kick_->init(&memory_,&textlog_);

  motion_ = new MotionModule();
  motion_->init(&memory_,&textlog_);

  sensor_ = new SensorModule();
  sensor_->init(&memory_,&textlog_);
  sonar_ = new SonarModule();
  sonar_->init(&memory_,&textlog_);
  getup_ = new GetupModule();
  getup_->init(&memory_,&textlog_);
  //kickside_=new KickSideMiddleModule();
  //kickside_->init(&memory_,&textlog_);
  //specialM_=new SpecialMotionModule();
  //specialM_->init(&memory_,&textlog_);

  if (USE_AL_MOTION)
    assert(WALK_TYPE == AL_WALK);
  else
    assert(WALK_TYPE != AL_WALK);

  switch (WALK_TYPE) {
    case AL_WALK:
      break;
    case HTWK_WALK:
      //htwk_walk_ = new HTWKWalkModule();
      //htwk_walk_->init(&memory_,&textlog_);
      break;
    case UT_NAO_DEVILS_WALK:
      walk_ = new WalkModule();
      walk_->init(&memory_,&textlog_);
      break;
    case BHUMAN_WALK:
      bh_walk_ = new BHWalkModule();
      bh_walk_->init(&memory_,&textlog_);
      kick_->setStand(bh_walk_->STAND_ANGLES);
      break;
  }
}

void MotionCore::initMemory() {
  // create all the modules that are used
  //MemorySource mem_source = MEMORY_SIM;
  //if (type_ == CORE_ROBOT)
    //mem_source = MEMORY_ROBOT;

  //Add required memory blocks
  memory_.addBlockByName("body_model");
  memory_.addBlockByName("graphable");
  memory_.addBlockByName("kick_engine");
  memory_.addBlockByName("walk_engine");
  memory_.addBlockByName("odometry");
  //memory_.addBlockByName("game_state");
  //memory_.addBlockByName("sonar");

  memory_.getOrAddBlockByName(frame_info_,"frame_info");
  // joint angles
  memory_.getBlockByName(raw_joint_angles_,"raw_joint_angles");
  memory_.getOrAddBlockByName(processed_joint_angles_,"processed_joint_angles");
  // commands
  memory_.getBlockByName(raw_joint_commands_,"raw_joint_commands");
  memory_.getOrAddBlockByName(processed_joint_commands_,"processed_joint_commands");
  // sensors
  memory_.getBlockByName(raw_sensors_,"raw_sensors");
  memory_.getOrAddBlockByName(processed_sensors_,"processed_sensors");

  memory_.getOrAddBlockByName(body_model_,"body_model");
  memory_.getOrAddBlockByName(walk_request_,"walk_request");
  memory_.getOrAddBlockByName(kick_params_,"kick_params");
  memory_.getOrAddBlockByName(walk_param_,"walk_param");
  memory_.getOrAddBlockByName(al_walk_param_,"al_walk_param");
  memory_.getOrAddBlockByName(walk_info_,"walk_info");

  memory_.getOrAddBlockByName(kick_request_,"kick_request");
  memory_.getOrAddBlockByName(odometry_,"odometry");
  memory_.getOrAddBlockByName(processed_sonar_,"processed_sonar");

  memory_.getOrAddBlockByName(robot_info_,"robot_info",MemoryOwner::SHARED);

  memory_.getOrAddBlockByName(world_objects_,"world_objects",MemoryOwner::SHARED);

  // synchronized blocks - the true means remove any existing locks
  memory_.getOrAddBlockByName(sync_body_model_,"sync_body_model",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_joint_angles_,"sync_joint_angles",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_kick_request_,"sync_kick_request",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_odometry_,"sync_odometry",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_sensors_,"sync_sensors",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_walk_request_,"sync_walk_request",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_joint_commands_,"sync_joint_commands",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_processed_sonar_,"sync_processed_sonar",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_kick_params_,"sync_kick_params",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_walk_param_,"sync_walk_param",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_al_walk_param_,"sync_al_walk_param",MemoryOwner::SYNC);
  memory_.getOrAddBlockByName(sync_walk_info_,"sync_walk_info",MemoryOwner::SYNC);

  // Read configuration file and place appropriate values in memory
  if (type_ == CORE_ROBOT) {
    Calibration calibration;
    std::cout << "Reading calibration file: " << memory_.data_path_ + "calibration.txt" << std::endl;
    if (calibration.readFromFile(memory_.data_path_ + "calibration.txt")) {
      // Disabled these because of new calibration system - JM 05/08/13
      //robot_info_->dimensions_.setCameraParameters(DEG_T_RAD*calibration.tilt_bottom_cam_, DEG_T_RAD*calibration.roll_bottom_cam_, DEG_T_RAD*calibration.tilt_top_cam_, DEG_T_RAD*calibration.roll_top_cam_);
      //robot_info_->dimensions_.setHeadOffsets(DEG_T_RAD * calibration.head_tilt_offset_, DEG_T_RAD * calibration.head_pan_offset_);
    }
  }

  // print out all the memory blocks we're using
  std::vector<std::string> memory_block_names;
  memory_.getBlockNames(memory_block_names,false);
  std::cout << "INITIAL MEMORY BLOCKS:" << std::endl;
  for (unsigned int i = 0; i < memory_block_names.size(); i++)
    std::cout << memory_block_names[i] << std::endl;
  std::cout << "--------------" << std::endl;

  textlog_.setFrameInfo(frame_info_);
}

void MotionCore::enableTextLogging(const char *filename) {
  if (filename) {
    textlog_.open(filename);
  } else {
    textlog_.open("motion", true);
  }
}

void MotionCore::disableTextLogging() {
  textlog_.close();
}

void MotionCore::setMemoryVariables() {

  // Set the data path for the data folder
  switch (type_) {
  case CORE_ROBOT:
    memory_.data_path_ = "/home/nao/data/";
    break;
  case CORE_SIM:
  case CORE_TOOL:
  default:
    memory_.data_path_ = (std::string(getenv("NAO_HOME")) + "/data/").c_str();
    break;
  }

  // Pass the coretype to the memory so that it can be accessed by different modules
  memory_.core_type_ = type_;
}

void MotionCore::preProcess() {
  const int *signs;
  if (frame_info_->source == MEMORY_ROBOT)
    signs = robot_joint_signs;
  else
    signs = spark_joint_signs;

  // apply signs to joint angles
  for (int i=0; i<NUM_JOINTS; i++) {
    processed_joint_angles_->prevValues_[i] = processed_joint_angles_->values_[i];
    processed_joint_angles_->values_[i] = signs[i] * raw_joint_angles_->values_[i];
    processed_joint_angles_->stiffness_[i] = raw_joint_angles_->stiffness_[i];
    processed_joint_angles_->changes_[i] = processed_joint_angles_->values_[i] - processed_joint_angles_->prevValues_[i];
  }

  // handle head offsets for reading
  processed_joint_angles_->values_[HeadTilt] += robot_info_->dimensions_.values_[RobotDimensions::headTiltOffset];
  processed_joint_angles_->values_[HeadYaw] += robot_info_->dimensions_.values_[RobotDimensions::headPanOffset];
}

void MotionCore::postProcess() {
  // process
  const int *signs;
  if (frame_info_->source == MEMORY_ROBOT)
    signs = robot_joint_signs;
  else
    signs = spark_joint_signs;

  // NOTE: we aren't doing anything right away, since sometimes what we want to do is bad
  float time_passed = frame_info_->seconds_since_start - time_motion_started_;
  if (time_passed < 1.0) {
    raw_joint_commands_->send_body_angles_ = false;
    raw_joint_commands_->send_arm_angles_ = false;
    raw_joint_commands_->send_head_pitch_angle_ = false;
    raw_joint_commands_->send_head_yaw_angle_ = false;
    raw_joint_commands_->send_stiffness_ = false;
    raw_joint_commands_->send_back_standup_ = false;
    raw_joint_commands_->send_sonar_command_ = false;
    return;
  }

  raw_joint_commands_->body_angle_time_ = processed_joint_commands_->body_angle_time_;
  raw_joint_commands_->arm_command_time_ = processed_joint_commands_->arm_command_time_;
  raw_joint_commands_->head_pitch_angle_time_ = processed_joint_commands_->head_pitch_angle_time_;
  raw_joint_commands_->head_yaw_angle_time_ = processed_joint_commands_->head_yaw_angle_time_;

  raw_joint_commands_->stiffness_time_ = processed_joint_commands_->stiffness_time_;
  raw_joint_commands_->send_back_standup_ = processed_joint_commands_->send_back_standup_;
/*
  // try to make the head hit its destination
  static float prevHeadTiltCommand = 0;
  static float prevHeadTilt = 0;
  static PIDController headTiltPID(1.0,0.1,0.0,DEG_T_RAD * 0.2);
  bool headTiltCommandChanged = (fabs(processed_joint_commands_->angles_[HeadTilt] - prevHeadTiltCommand) > 0.01);
  bool headTiltChanged = (fabs(processed_joint_angles_->values_[HeadTilt] - prevHeadTilt) > DEG_T_RAD * 0.5);
  prevHeadTiltCommand = processed_joint_commands_->angles_[HeadTilt];
  prevHeadTilt = processed_joint_angles_->values_[HeadTilt];
  if ((processed_joint_angles_->stiffness_[HeadTilt] < 1e-5) || headTiltCommandChanged || headTiltChanged) {
    headTiltPID.reset();
  } else {
    processed_joint_commands_->angles_[HeadTilt] += headTiltPID.update(processed_joint_angles_->values_[HeadTilt],processed_joint_commands_->angles_[HeadTilt]);
    //std::cout << RAD_T_DEG * processed_joint_angles_->values_[HeadTilt] << std::endl;
    headTiltPID.cropCumulative(DEG_T_RAD * 2.0);
  }
*/
  // apply signs to joint commands
  for (int i=0; i<NUM_JOINTS; i++) {
    raw_joint_commands_->angles_[i] = signs[i] * processed_joint_commands_->angles_[i];
  }

  // handle head offsets for commands
  if (!processed_joint_commands_->head_pitch_angle_change_)
    raw_joint_commands_->angles_[HeadTilt] -= signs[HeadTilt] * robot_info_->dimensions_.values_[RobotDimensions::headTiltOffset];
  if (!processed_joint_commands_->head_yaw_angle_change_)
    raw_joint_commands_->angles_[HeadYaw] -= signs[HeadYaw] * robot_info_->dimensions_.values_[RobotDimensions::headPanOffset];


  // handle stiffness
  if (processed_joint_commands_->send_stiffness_) {
    for (int i=0; i<NUM_JOINTS; i++) {
      raw_joint_commands_->stiffness_[i] = processed_joint_commands_->stiffness_[i];
    }
  }
  raw_joint_commands_->send_stiffness_ = processed_joint_commands_->send_stiffness_;

  raw_joint_commands_->send_body_angles_ = processed_joint_commands_->send_body_angles_;
  raw_joint_commands_->send_arm_angles_ = processed_joint_commands_->send_arm_angles_;
  raw_joint_commands_->send_head_pitch_angle_ = processed_joint_commands_->send_head_pitch_angle_;
  raw_joint_commands_->send_head_yaw_angle_ = processed_joint_commands_->send_head_yaw_angle_;

  raw_joint_commands_->head_pitch_angle_change_ = processed_joint_commands_->head_pitch_angle_change_;
  raw_joint_commands_->head_yaw_angle_change_ = processed_joint_commands_->head_yaw_angle_change_;

  raw_joint_commands_->send_sonar_command_ = processed_joint_commands_->send_sonar_command_;
  raw_joint_commands_->sonar_command_ = processed_joint_commands_->sonar_command_;
}

void MotionCore::publishData() {
  memory_.motion_vision_lock_->lock();

  *sync_body_model_ = *body_model_;
  *sync_joint_angles_ = *processed_joint_angles_;
  *sync_sensors_ = *processed_sensors_;
  *sync_odometry_ = *odometry_;
  
  *sync_kick_request_ = *kick_request_;
  sync_kick_request_->kick_running_ = kick_request_->kick_running_;
  sync_kick_request_->finished_with_step_ = kick_request_->finished_with_step_;
/*
  sync_kick_request_->ball_seen_ = kick_request_->ball_seen_;
  sync_kick_request_->ball_image_center_x_ = kick_request_->ball_image_center_x_;
  sync_kick_request_->ball_image_center_y_ = kick_request_->ball_image_center_y_;
  sync_kick_request_->ball_rel_x_ = kick_request_->ball_rel_x_;
  sync_kick_request_->ball_rel_y_ = kick_request_->ball_rel_y_;
  sync_kick_request_->goal_seen_ = kick_request_->goal_seen_;
  sync_kick_request_->goal_image_center_x_ = kick_request_->goal_image_center_x_;
  sync_kick_request_->goal_image_center_y_ = kick_request_->goal_image_center_y_;
  sync_kick_request_->goal_rel_x_ = kick_request_->goal_rel_x_;
  sync_kick_request_->goal_rel_y_ = kick_request_->goal_rel_y_;
*/

  *sync_walk_request_ = *walk_request_;
  sync_walk_request_->new_command_ = false;
  sync_walk_request_->slow_stand_ = false;
  sync_walk_request_->set_kick_step_params_ = false;

  *sync_processed_sonar_ = *processed_sonar_;
  *sync_walk_info_ = *walk_info_;

  memory_.motion_vision_lock_->unlock();
}

void MotionCore::receiveData() {
  memory_.motion_vision_lock_->lock();

  *kick_request_ = *sync_kick_request_;
  *walk_request_ = *sync_walk_request_;
  //std::cout << "mot: " << walk_request_->step_into_kick_ << " " << walk_request_->perform_kick_ << std::endl;
  *processed_joint_commands_ = *sync_joint_commands_;
  *odometry_ = *sync_odometry_;

  if (sync_kick_params_->send_params_) {
    *kick_params_ = *sync_kick_params_;
    sync_kick_params_->send_params_ = false;
  }
  if (sync_walk_param_->send_params_) {
    *walk_param_ = *sync_walk_param_;
    sync_walk_param_->send_params_ = false;
  }
  if (sync_al_walk_param_->send_params_){
    *al_walk_param_ = *sync_al_walk_param_;
    sync_al_walk_param_->send_params_ = false;
  }

  memory_.motion_vision_lock_->unlock();
}


void MotionCore::updateOdometry(){
  // TODO RE-ENABLE ODOMETRY
/*
  // set if standing or walking
  odometry_->standing = !walk_info_->walk_is_active_;
  if (odometry_->standing) {
    last_stand_frame_ = frame_info_->frame_id;
  }
  if (walk_request_->motion_ == WalkRequestBlock::STAND) {
    if (next_stand_frame_ == 0)
      next_stand_frame_ = frame_info_->frame_id + 4 * al_walk_param_->walk_step_min_period_;
  } else
    next_stand_frame_ = 0;

  // save where robot was last time vision updated odom
  if (odometry_->displacement.translation.x == 0 && odometry_->displacement.translation.y == 0 && odometry_->displacement.rotation == 0){
    walk_info_->robot_odometry_frame_ = walk_info_->robot_last_position_;
    last_odometry_update_ = frame_info_->frame_id;
  }

  // save last position
  walk_info_->robot_last_position_ = walk_info_->robot_position_;

  // odometry is how much torso has moved last time vision used it
  odometry_->displacement = walk_info_->robot_position_.globalToRelative(walk_info_->robot_odometry_frame_);

  // correct for the robot walk offsets
  bool do_odometry_correction = ((!odometry_->standing) && (frame_info_->frame_id - last_stand_frame_ > al_walk_param_->walk_step_min_period_ * 4)); // is not standing, and not just coming out of a stand
  do_odometry_correction = do_odometry_correction && ((frame_info_->frame_id < next_stand_frame_) || (next_stand_frame_ == 0)); // don't correct if we're in the last step before standing

  //std::cout << next_stand_frame_ << " " << frame_info_->frame_id << std::endl;
  if (do_odometry_correction) {
    // TODO: currently ignoring other offsets
    float steps_per_second = 50.0 / al_walk_param_->walk_step_min_period_;
    float max_turn_speed = DEG_T_RAD * al_walk_param_->walk_max_step_theta_ * steps_per_second;
    float turn_offset_vel = walk_request_->odometry_turn_offset_ * max_turn_speed;
    unsigned int num_frames_passed = frame_info_->frame_id - last_odometry_update_ + 1; // +1 for the current frame
    double time = 0.01 * num_frames_passed;
    float da = -1 * turn_offset_vel * time;

    //float da = -1 * walk_request_->odometry_turn_offset_ * DEG_T_RAD * al_walk_param_->walk_max_step_theta_ / (2.0 * al_walk_param_->walk_step_min_period_);
    //da *= 4;
    //std::cout << odometry_->displacement.rotation << " " << da << std::endl;
    odometry_->displacement.rotation += da;
  }

  // multiply by factor
  odometry_->displacement.translation.x *= al_walk_param_->fwd_odometry_factor_;
  odometry_->displacement.translation.y *= al_walk_param_->side_odometry_factor_;
  if (odometry_->displacement.rotation > 0)
    odometry_->displacement.rotation *= al_walk_param_->turn_ccw_odometry_factor_;
  else
    odometry_->displacement.rotation *= al_walk_param_->turn_cw_odometry_factor_;
*/
}
