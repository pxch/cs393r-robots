#include "UTMainWnd.h"

#include <ctime>

// windows
#include "FilesWindow.h"
#include "LogEditorWindow.h"
#include "LogSelectWindow.h"
#include "LogWindow.h"
#include "MotionWindow.h"
#include "PlotWindow.h"
#include "VisionWindow.h"
#include "WorldWindow.h"
#include "JointsWindow.h"
#include "WalkWindow.h"
#include "StateWindow.h"
#include "CameraWindow.h"
#include "SensorWindow.h"
#include "TeamConfigWindow.h"

//Networking
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <common/WorldObject.h>
#include <common/RobotInfo.h>

#include <memory/PrivateMemory.h>
#include <memory/LogReader.h>
#include <memory/Memory.h>

// memory blocks
#include <memory/BodyModelBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/ImageBlock.h>
#include <memory/SensorBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/SimEffectorBlock.h>

#include <common/Config.h>
#include <common/Calibration.h>

#include <opponents/OppModule.h>
#include <lua/LuaModule.h>
#include <python/PythonModule.h>

#include <math/Pose3D.h>
#include <math/RotationMatrix.h>
#include <math/Vector3.h>

UTMainWnd::UTMainWnd(const char *filename, bool core):
  memory_log_(0),
  startFrame(0),
  endFrame(-1),
  io_service(),
  acceptor(io_service,tcp::endpoint(tcp::v4(),TOOL_TCP_PORT)),
  stream_memory_(false,MemoryOwner::TOOL_MEM, 0, 1),
  filesWnd_(NULL),
  logEditorWnd_(NULL),
  logSelectWnd_(NULL),
  logWnd_(NULL),
  motionWnd_(NULL),
  plotWnd_(NULL),
  visionWnd_(NULL),
  worldWnd_(NULL),
  jointsWnd_(NULL),
  walkWnd_(NULL),
  stateWnd_(NULL),
  cameraWnd_(NULL),
  sensorWnd_(NULL),
  teamWnd_(NULL),
  rcPath(QString(getenv("NAO_HOME")) + "/data/.utnaotoolrc")
{

  setupUi(this);

  memory_ = NULL;
  logWnd_ = new LogWindow(this);
  motionWnd_ = new MotionWindow();
  plotWnd_ = new PlotWindow();
  worldWnd_ = new WorldWindow(this);
  jointsWnd_ = new JointsWindow();
  walkWnd_ = new WalkWindow();
  stateWnd_ = new StateWindow();
  cameraWnd_ = new CameraWindow(this);
  sensorWnd_ = new SensorWindow();
  teamWnd_ = new TeamConfigWindow(this);

  std::vector<std::string> block_names;
  visionCore_ = new VisionCore(CORE_TOOL,false,0,1);
  visionWnd_ = new VisionWindow(this, visionCore_);
  visionCore_->memory_->getBlockNames(block_names,false);
  logEditorWnd_ = new LogEditorWindow(this);
  logSelectWnd_ = new LogSelectWindow(this,block_names);
  filesWnd_ = new FilesWindow(this);
  
  connect (this, SIGNAL(newStreamFrame()), this, SLOT(handleStreamFrame()));

  connect (frameSlider, SIGNAL(valueChanged(int)), this, SLOT(gotoSnapshot(int)) );
  connect (currentFrameSpin, SIGNAL(valueChanged(int)), this, SLOT(gotoSnapshot(int)) );

  connect (jointsButton, SIGNAL(clicked()), this, SLOT(openJointsWnd()) );
  connect (walkButton, SIGNAL(clicked()), this, SLOT(openWalkWnd()));
  connect (sensorButton, SIGNAL(clicked()), this, SLOT(openSensorWnd()) );
  connect (motionButton, SIGNAL(clicked()), this, SLOT(openMotionWnd()) );
  connect (plotButton, SIGNAL(clicked()), this, SLOT(openPlotWnd()) );
  connect (filesButton, SIGNAL(clicked()), this, SLOT(openFilesWnd()) );
  connect (fullVisionButton, SIGNAL(clicked()), this, SLOT(openVisionWnd()) );
  connect (worldButton, SIGNAL(clicked()), this, SLOT(openWorldWnd()) );
  connect (logButton, SIGNAL(clicked()), this, SLOT(openLogWnd()) );
  connect (logEditorButton, SIGNAL(clicked()), this, SLOT(openLogEditorWnd()) );
  connect (logSelectButton, SIGNAL(clicked()), this, SLOT(openLogSelectWnd()) );
  connect (stateButton, SIGNAL(clicked()), this, SLOT(openStateWnd()) );
  connect (cameraButton, SIGNAL(clicked()), this, SLOT(openCameraWnd()) );
  connect (teamButton, SIGNAL(clicked()), this, SLOT(openTeamWnd()) );

  connect (motionWnd_->robot_, SIGNAL(prevSnapshot()), this, SLOT(prevSnapshot()) );
  connect (motionWnd_->robot_, SIGNAL(nextSnapshot()), this, SLOT(nextSnapshot()) );
  connect (motionWnd_->robot_, SIGNAL(play()), this, SLOT(play()) );
  connect (motionWnd_->robot_, SIGNAL(pause()), this, SLOT(pause()) );

  connect (worldWnd_->world, SIGNAL(prevSnapshot()), this, SLOT(prevSnapshot()) );
  connect (worldWnd_->world, SIGNAL(nextSnapshot()), this, SLOT(nextSnapshot()) );
  connect (worldWnd_->world, SIGNAL(play()), this, SLOT(play()) );
  connect (worldWnd_->world, SIGNAL(pause()), this, SLOT(pause()) );

  connect (logWnd_, SIGNAL(prevSnapshot()), this, SLOT(prevSnapshot()) );
  connect (logWnd_, SIGNAL(nextSnapshot()), this, SLOT(nextSnapshot()) );

  connect (plotWnd_, SIGNAL(gotoSnapshot(int)), this, SLOT(gotoSnapshot(int)) );
  connect (plotWnd_, SIGNAL(prevSnapshot()), this, SLOT(prevSnapshot()) );
  connect (plotWnd_, SIGNAL(nextSnapshot()), this, SLOT(nextSnapshot()) );
  connect (plotWnd_, SIGNAL(play()), this, SLOT(play()) );
  connect (plotWnd_, SIGNAL(pause()), this, SLOT(pause()) );

  connect (visionWnd_, SIGNAL(prevSnapshot()), this, SLOT(prevSnapshot()) );
  connect (visionWnd_, SIGNAL(nextSnapshot()), this, SLOT(nextSnapshot()) );
  connect (visionWnd_, SIGNAL(play()), this, SLOT(play()) );
  connect (visionWnd_, SIGNAL(pause()), this, SLOT(pause()) );
  connect (visionWnd_, SIGNAL(setCore(bool)), this, SLOT(setCore(bool)) );
  connect (this, SIGNAL(setStreaming(bool)), visionWnd_, SLOT(setStreaming(bool)) );
  connect (this, SIGNAL(newLogFrame(int)), visionWnd_, SLOT(handleNewLogFrame(int)));
  connect (this, SIGNAL(newLogLoaded(Log*)), visionWnd_, SLOT(handleNewLogLoaded(Log*)));
  connect (this, SIGNAL(newStreamFrame()), visionWnd_, SLOT(handleNewStreamFrame()));
  connect (this, SIGNAL(runningCore()), visionWnd_, SLOT(handleRunningCore()));

  connect (actionOpen_Log, SIGNAL(triggered()), this, SLOT(openLog()) );
  connect (actionOpen_Recent_Log, SIGNAL(triggered()), this, SLOT(openRecent()));
  connect (actionRe_Run_Core, SIGNAL(triggered()), this, SLOT(rerunCore()));

  connect (runCoreRadio, SIGNAL(clicked()), this, SLOT(runCore()) );
  connect (viewLogRadio, SIGNAL(clicked()), this, SLOT(runLog()) );
  connect (streamRadio, SIGNAL(clicked()), this, SLOT(runStream()) );

  current_index_ = -1;
  runningCore_ = false;
  coreAvailable_ = false;
  isStreaming_ = false;

  currentFrameSpin->setRange(0,0);
  frameSlider->setRange(0,0);

  //tcpServer.listen(QHostAddress::Any,TOOL_TCP_PORT);
  //connect (&tcpServer, SIGNAL(newConnection()), this, SLOT(tcpConnection()) );
  boost::thread t(boost::bind(server,this));

  if (filename) {
    loadLog(filename);
    if (core) {
      runningCore_ = true;
      runCore();
    }
  }

  loadConfig();
}

void UTMainWnd::handleStreamFrame() {
  gotoSnapshot(0);
}

void server(UTMainWnd *main) {
  for (;;) {
    socket_ptr sock(new tcp::socket(main->io_service));
    main->acceptor.accept(*sock);
    boost::thread t(boost::bind(session,main,sock));
  }
}

void session(UTMainWnd *main,socket_ptr sock) {
  main->processStream(sock);
}

void UTMainWnd::processStream(socket_ptr sock) {
  std::size_t ret;
  unsigned long &send_len = stream_msg_.send_len_;
  std::size_t expected_len = sizeof(send_len);
  try {
    for (;;) {
      ret = boost::asio::read(*sock,boost::asio::buffer(&send_len,expected_len));
      if (ret != expected_len) {
        std::cout << "Couldn't read send_len " << ret << " " << expected_len << std::endl << std::flush;
        return;
      }
      if (send_len > MAX_STREAMING_MESSAGE_LEN) {
        std::cout << "MESSAGE TOO LARGE " << send_len << " " << MAX_STREAMING_MESSAGE_LEN << std::endl << std::flush;
        return;
      }
      ret = boost::asio::read(*sock,boost::asio::buffer(&(stream_msg_.orig_len_),expected_len));
      if (ret != expected_len) {
        std::cout << "Couldn't read orig_len " << ret << " " << expected_len << std::endl << std::flush;
        return;
      }
      ret = boost::asio::read(*sock,boost::asio::buffer(&stream_msg_.data_,send_len - 2 * expected_len));
      char *msg = (char*)stream_msg_.postReceive(ret);
      if (msg == NULL) {
        std::cout << "Invalid tcp message" << std::endl << std::flush;
        return;
      }
      StreamBuffer sb(msg, stream_msg_.orig_len_);
      LogReader stream_reader(sb);
      bool res = stream_reader.readMemory(stream_memory_);
      if (!res) {
        std::cout << "Problem reading memory from tcp message" << std::endl;
        return;
      }
      delete []msg;
      emit newStreamFrame();
    }
  } catch (boost::system::system_error) {
    std::cout << "Error reading from tcp, disconnecting" << std::endl << std::flush;
    return;
  }
}

UTMainWnd::~UTMainWnd() {
  delete visionCore_;
}

QString UTMainWnd::getCurrentAddress() {
  if (filesWnd_ == NULL) return "";
  return filesWnd_->locationBox->currentText();
}

#include <memory/WalkRequestBlock.h>

void UTMainWnd::loadLog(const char *filename) {
  std::cout << "Loading " << filename << std::endl;
  LogReader reader(filename);
  if(memory_log_) delete memory_log_;
  memory_log_ = reader.readLog(startFrame, endFrame);
  memory_log_->name = std::string(filename);

  int size = memory_log_->size();
  std::cout << "Loaded " << size << " memory frames\n" << std::flush;
  if (size == 0)
    return;

  // load the corresponding text file as well
  logFile = std::string(filename);
  setCore(false);
  logWnd_->loadTextFile(filename); // done in setCore // not always apparently

  numFrameEdit->setText(QString::number(size-1));
  currentFrameSpin->setRange(0,size-1);
  frameSlider->setRange(0,size-1);

  // update plot window with whole log if we're using it
  if (plotWnd_->isVisible()){
    plotWnd_->setMemoryLog(memory_log_);
  }

  current_index_ = -1;

  core_log_.clear();
  coreAvailable_ = false;
  if (runningCore_) {
    runCore();
  }

  gotoSnapshot(0);
  emit newLogLoaded(memory_log_);
}


void UTMainWnd::rerunCore() {
  coreAvailable_ = false;
  runningCore_ = false;
  runCore();
}


void UTMainWnd::runCore() {
  stopStream();
  // Check if we have more than one log
  if (memory_log_ == NULL || memory_log_->size() == 0) {
    std::cout << "No log available to run core on" << std::endl;
    return;
  }

  // Run core if core is not available
  if (!coreAvailable_) {

    // Remove any previous log that was there
    remove("core.txt");
    core_log_.clear();

    bool locOnly = false;
    bool visOnly = false;
    if (localizationOnlyCheckBox->isChecked()){
      locOnly = true;
      visOnly = false;
      cout << "Run core will only run localization" << endl;
    }
    else if (visionOnlyCheckBox->isChecked()){
      locOnly = false;
      visOnly = true;
      cout << "Run core will only run vision" << endl;
    }
    else {
      cout << "Run core runs all of vision core (vis, loc, behavior, etc)" << endl;
    }

    // Copy over memory from the log
    for (unsigned i = 0; i < memory_log_->size(); i++) {
      core_log_.push_back((*memory_log_)[i]);
    }

    // restart lua
    if(visionCore_->lua_) visionCore_->lua_->startLua();
    if(visionCore_->python_) visionCore_->python_->startPython();
    // reinitialize UKFs as they do not keep their matrices in memory
    // disabled for 393r-f2013 visionCore_->localization_->reInit();

    visionCore_->opponents_->reInit();

    for (unsigned i = 0; i < core_log_.size(); i++) {
      // Note: we do not need to copy any memory all anything as the state is maintained
      // internally by loc and vision is state-free
      visionCore_->updateMemory(&core_log_[i],locOnly);
      if (i == 0) {
        // load either sim or robot color tables based on the first frame of the log
        if (!locOnly)
          visionCore_->vision_->loadColorTables();
        // disabled for 393r-f2013 visionCore_->localization_->resetTimeLast();
        //                         visionCore_->localization_->initFromMemory();
        visionCore_->enableTextLogging("core.txt");
      }
      if (locOnly){
        visionCore_->localization_->processFrame();
        visionCore_->opponents_->processFrame();
      } else if (visOnly) {
        visionCore_->vision_->processFrame();
      } else {
        visionCore_->processVisionFrame();
      }

      /*
      // copy raw image back over
      ImageBlock* orig_raw = NULL;
      memory_log_[i].getBlockByName(orig_raw, "raw_image", false);
      ImageBlock* copy_raw = NULL;
      core_log_[i].getBlockByName(copy_raw, "raw_image");
      if (orig_raw != NULL){
        *copy_raw = *orig_raw;
      } else {
        cout << " set core mem to have null pointers for raw images" << endl;
        copy_raw->img_top_ = NULL;
        copy_raw->img_bottom_ = NULL;
      }
      */
    }

    visionCore_->disableTextLogging();

    coreAvailable_ = true;
  }

  if (!runningCore_) {
    runningCore_ = true;
    emit runningCore();
    logWnd_->loadTextFile("core.txt");
    int index = current_index_;
    current_index_ = -1;
    int size = core_log_.size();
    numFrameEdit->setText(QString::number(size-1));
    currentFrameSpin->setRange(0,size-1);
    frameSlider->setRange(0,size-1);
    gotoSnapshot(index);
  }
}

void UTMainWnd::runLog() {
  stopStream();
  if (runningCore_) {
    runningCore_ = false;
    logWnd_->loadTextFile(logFile.c_str());
    int index = current_index_;
    current_index_ = -1;
    int size = memory_log_->size();
    numFrameEdit->setText(QString::number(size-1));
    currentFrameSpin->setRange(0,size-1);
    frameSlider->setRange(0,size-1);
    gotoSnapshot(index);
  }
}

void UTMainWnd::setCore(bool value) {
  if (value) {
    this->runCoreRadio->setChecked(true);
    runCore();
  }
  else {
    this->viewLogRadio->setChecked(true);
    runLog();
  }
}

void UTMainWnd::runStream() {
  runningCore_ = false;
  isStreaming_ = true;
  sendUDPCommandToCurrent("XB");
  emit setStreaming(true);
}

void UTMainWnd::stopStream() {
  if (isStreaming_)
    sendUDPCommandToCurrent("XE");
  isStreaming_ = false;
  emit setStreaming(false);
}

void UTMainWnd::gotoSnapshot(int index) {
  // do nothing if we're already at this snapshot
  if (!isStreaming_ && ((uint16_t)index == current_index_))
    return;
  emit newLogFrame(index);

  current_index_= (uint16_t)index;
  currentFrameSpin->setValue(current_index_);
  frameSlider->setValue(current_index_);

  if (runningCore_) {
    if (current_index_ >= core_log_.size())
      current_index_ = core_log_.size() - 1;
    memory_ = &(core_log_[current_index_]);
  } else if (isStreaming_) {
    memory_ = &stream_memory_;
  } else{
    if (current_index_ >= memory_log_->size())
      current_index_ = memory_log_->size() - 1;
    memory_ = &((*memory_log_)[current_index_]);
  }

  if (motionWnd_->isVisible())
    motionWnd_->update(memory_);
  if (plotWnd_->isVisible())
    plotWnd_->update(memory_);
  if (visionWnd_->isVisible())
    visionWnd_->update(memory_);
  if (worldWnd_->isVisible())
    worldWnd_->updateMemory(memory_);
  if (logWnd_->isVisible())
    logWnd_->updateFrame(memory_);
  if (jointsWnd_->isVisible())
    jointsWnd_->update(memory_);
  if (walkWnd_->isVisible())
    walkWnd_->update(memory_);
  if (stateWnd_->isVisible())
    stateWnd_->update(memory_);
  if (cameraWnd_->isVisible())
    cameraWnd_->update(memory_);
  if (sensorWnd_->isVisible())
    sensorWnd_->update(memory_);
}

void UTMainWnd::prevSnapshot() {
  int index = current_index_ - 1;
  if (index < 0) index=0;
  gotoSnapshot(index);
}

void UTMainWnd::nextSnapshot() {
  if (isStreaming_) return;
  unsigned int index = current_index_ + 1;
  if (index > memory_log_->size()-1) index=memory_log_->size()-1;
  gotoSnapshot(index);
}


void UTMainWnd::play() {
}

void UTMainWnd::pause() {
}

bool UTMainWnd::openLog() {
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open Log File"),
                                                  QString(getenv("NAO_HOME")) + "/logs/",
                                                  tr("UT Nao Logs (*.log)"));
  if (fileName == NULL)
    return false;
  loadLog(fileName.toLatin1());
  return true;
}

bool UTMainWnd::openRecent() {

  // find most recent log
  QDir* logDir = new QDir(QString(getenv("NAO_HOME")) + "/logs/");
  QStringList filters;
  filters << "*.log";
  logDir->setNameFilters(filters);
  logDir->setSorting(QDir::Time);
  QFileInfoList files = logDir->entryInfoList();
  if (files.size() == 0){
    cout << "No logs" << endl;
    return false;
  }
  QFileInfo file = files.first();
  QString filename = file.filePath();
  //cout << "File: " << filename.toLatin1() << endl;
  loadLog(filename.toLatin1());
  return true;
}


void UTMainWnd::openFilesWnd() {
  filesWnd_->show();
  filesWnd_->raise();
  filesWnd_->activateWindow();
}

void UTMainWnd::openTeamWnd() {
  teamWnd_->show();
  teamWnd_->raise();
  teamWnd_->activateWindow();
}

void UTMainWnd::openLogSelectWnd() {
  logSelectWnd_->show();
  logSelectWnd_->raise();
  logSelectWnd_->activateWindow();
}

void UTMainWnd::openLogEditorWnd() {
  logEditorWnd_->show();
  logEditorWnd_->raise();
  logEditorWnd_->activateWindow();
}

void UTMainWnd::openLogWnd() {
  logWnd_->show();
  logWnd_->raise();
  logWnd_->activateWindow();
  if (memory_) logWnd_->updateFrame(memory_);
}

void UTMainWnd::openMotionWnd() {
  motionWnd_->show();
  motionWnd_->raise();
  motionWnd_->activateWindow();
  if (memory_) motionWnd_->update(memory_);
}

void UTMainWnd::openPlotWnd() {
  plotWnd_->setMemoryLog(memory_log_);
  plotWnd_->show();
  plotWnd_->activateWindow();
  if (memory_) plotWnd_->update(memory_);
}

void UTMainWnd::openVisionWnd() {
  visionWnd_->show();
  visionWnd_->raise();
  visionWnd_->activateWindow();
  if (memory_) visionWnd_->update(memory_);
}

void UTMainWnd::openWorldWnd() {
  worldWnd_->show();
  worldWnd_->raise();
  worldWnd_->activateWindow();
  if (memory_) worldWnd_->updateMemory(memory_);
}

void UTMainWnd::openJointsWnd() {
  jointsWnd_->show();
  jointsWnd_->raise();
  jointsWnd_->activateWindow();
  if (memory_) jointsWnd_->update(memory_);
}

void UTMainWnd::openWalkWnd() {
  walkWnd_->show();
  walkWnd_->raise();
  walkWnd_->activateWindow();
  if (memory_) walkWnd_->update(memory_);
}

void UTMainWnd::openSensorWnd() {
  sensorWnd_->show();
  sensorWnd_->raise();
  sensorWnd_->activateWindow();
  if (memory_) sensorWnd_->update(memory_);
}

void UTMainWnd::openStateWnd() {
  stateWnd_->show();
  stateWnd_->raise();
  stateWnd_->activateWindow();
  if (memory_) stateWnd_->update(memory_);
}

void UTMainWnd::openCameraWnd() {
  cameraWnd_->show();
  cameraWnd_->raise();
  cameraWnd_->activateWindow();
  if (memory_) cameraWnd_->update(memory_);
}
void UTMainWnd::remoteRestartLua() {
  sendUDPCommandToCurrent("R");
}

void UTMainWnd::updateConfigFile() {

  if (getCurrentAddress() == "localhost") { // Not required for simulator
    cout << "Error: config file is not supported for simulator!" << endl;
    return;
  }

  Config config;
  config.robot_id_ = getCurrentAddress().section('.', 3).toInt();
  config.team_ = filesWnd_->teamNumBox->value();
  config.role_ = filesWnd_->roleBox->value();

  if (config.team_ != 23){
    cout << "WARNING! Setting team # to " << config.team_ << " while our RC2011 team number is 23" << endl;
  }

  // Write file to set values
  config.writeToFile("./config.txt");

  // Send the file to the robot
  QString cmd = "scp ";
  cmd += "./config.txt ";
  cmd += "nao@" + getCurrentAddress() + ":~/data/config.txt ";
  std::cout << "Executing command: " << cmd.toStdString() << std::endl;
  QProcess processSend;
  processSend.start(cmd);
  if (!processSend.waitForStarted()) {
    cout << "Error: unable to copy config.txt to " << getCurrentAddress().toStdString() << endl << flush;
    return;
  }
  processSend.waitForFinished();
  std::cout << "Done!" << std::endl;

}

void UTMainWnd::updateCalibrationFile() {

  if (getCurrentAddress() == "localhost") { // Not required for simulator
    cout << "Error: config file is not supported for simulator!" << endl;
    return;
  }

  // Send the file to the robot
  filesWnd_->sendFile("./calibration.txt","~/data","calibration.txt");
  filesWnd_->sendFile(filesWnd_->dataPath + "defaultcamera.cal","~/data","defaultcamera.cal");
  QString id = getCurrentAddress().split(".")[3];
  filesWnd_->sendFile(filesWnd_->dataPath + id + "camera.cal","~/data",id+"camera.cal");
  //QString cmd = "scp ";
  //cmd += "./calibration.txt ";
  //cmd += "nao@" + getCurrentAddress() + ":~/data/calibration.txt ";
  //std::cout << "Executing command: " << cmd.toStdString() << std::endl;
  //QProcess processSend;
  //processSend.start(cmd);
  //if (!processSend.waitForStarted()) {
    //cout << "Error: unable to copy calibration.txt to " << getCurrentAddress().toStdString() << endl << flush;
    //return;
  //}
  //processSend.waitForFinished();
  //std::cout << "Done!" << std::endl;
}

void UTMainWnd::readCalibrationFile() {
  // Copy over the file from the robot first to not overwrite useful values
  QString cmd = "scp ";
  cmd += "nao@" + getCurrentAddress() + ":~/data/calibration.txt ";
  cmd += "./calibration.txt";
  std::cout << "Executing command: " << cmd.toStdString() << std::endl;
  QProcess processGet;
  processGet.start(cmd);
  if (!processGet.waitForStarted()) {
    cout << "Error: unable to copy calibration.txt from " << getCurrentAddress().toStdString() << endl << flush;
  }
  processGet.waitForFinished();

  // Read file to get values
  Calibration calibration;
  if (calibration.readFromFile("./calibration.txt")) {
    //visionWnd_->updateCalibration(calibration.tilt_top_cam_, calibration.roll_top_cam_, calibration.tilt_bottom_cam_, calibration.roll_bottom_cam_, calibration.head_pan_offset_, calibration.head_tilt_offset_);
  } else {
    cout << "Error: unable to read calibration.txt from " << getCurrentAddress().toStdString() << endl << flush;
  }
}


void UTMainWnd::sendUDPCommand(QString address, QString cmd) {
  int sock, length, n;
  struct sockaddr_in server;
  struct hostent *hp;
  //char buffer[256];
  sock= socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    cout << "Error: creating socket\n" << endl;
    return;
  }
  server.sin_family = AF_INET;
  hp = gethostbyname(address.toLatin1().data());
  if (hp==0) {
    cout << "Error: Unknown host\n" << flush;
    return;
  }
  bcopy((char *)hp->h_addr,(char *)&server.sin_addr,hp->h_length);
  server.sin_port = htons(TOOL_UDP_PORT);
  length=sizeof(struct sockaddr_in);
  //bzero(buffer,256);
  //buffer=cmd.;
  n=sendto(sock,cmd.toLatin1().data(),strlen(cmd.toLatin1().data()),0,(sockaddr*)&server,length);
  if (n < 0) cout << "Error sending UDP command. Make sure you're connected to the robot's subnet.\n" << flush;
  //core->startLua();
}

void UTMainWnd::sendUDPCommandToCurrent(QString cmd) {
  QString address = getCurrentAddress();
  sendUDPCommand(address,cmd);
}
  
void UTMainWnd::loadConfig() {
  QFile savedFile(rcPath);
  if (savedFile.open(QIODevice::ReadOnly) ) {       
    QTextStream t( &savedFile );        // use a text stream
    filesWnd_->loadConfig(t);
    logWnd_->loadConfig(t);
    savedFile.close();
  }
}

void UTMainWnd::saveConfig() {
  QFile savedFile(rcPath);
  if (savedFile.open(QIODevice::WriteOnly) ) {       
    QTextStream t( &savedFile );        // use a text stream
    filesWnd_->saveConfig(t);
    logWnd_->saveConfig(t);
    savedFile.close();
  }
}

int UTMainWnd::loadInt(QTextStream &t, bool &ok) {
  QString line = t.readLine();
  if (line.isNull()) {
    ok = false;
    return 0;
  }
  return line.toInt(&ok);
}
