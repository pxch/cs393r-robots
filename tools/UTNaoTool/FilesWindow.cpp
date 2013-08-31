#include <QtGui>
#include "FilesWindow.h"
#include <iostream>
#include "UTMainWnd.h"
#include <stdio.h>
#include "LogSelectWindow.h"

using namespace std;

FilesWindow::FilesWindow(QMainWindow* p) {
  setupUi(this);
  setWindowTitle(tr("Files Window"));
  basePath = QString(getenv("NAO_HOME")) + "/";
  logPath = basePath + "logs/";
  dataPath = basePath + "data/";
  luaPath = basePath + "core/lua/";

  // luaDir = new QDir(luaPath);
  // luaWatch = new QFileSystemWatcher(this);
  // QStringList filters;
  // filters << "*.lua";
  // luaDir->setNameFilters(filters);


  // // add lua files to filewatch
  // foreach (QFileInfo info, luaDir->entryInfoList()) {
  //  luaWatch->addPath (info.absoluteFilePath() );
  // }
  // luaWatch->addPath (luaPath);
  // //
  // // add to the display
  // QStringList ls= luaDir->entryList();
  // QVBoxLayout *vbox = new QVBoxLayout();
  // numFiles = ls.size();
  // files = new QCheckBox[numFiles];
  // for (int i = 0; i < numFiles; ++i) {
  //   files[i].setText(ls.at(i).toLocal8Bit().constData());
  //   vbox->addWidget(&files[i]);
  // }
  // vbox->addStretch(1);
  // luaGroupBox->setLayout(vbox);
  //

  // 1 is our team number for RC 2013
  teamNumBox->setValue(1);

  prevTime=prevTime.currentDateTime();

  parent = p;
  robotStatus->setText("Dead");

  enableButtons(true);
  checkStatus();

  // start timer for status updates
  statusTimer = new QTimer(this);
  // check on robots using fping every 10 seconds (now variable based on whether ping succeeded)
	statusTimer->singleShot(10000,this,SLOT(checkStatus()));

  // until we get simulator back
  locationBox->addItem("localhost");
  locationBox->addItem("core");
  std::string ipList = std::string(getenv("NAO_HOME")) + "/data/ipList.txt";
  QFile file(ipList.c_str()); // List of IP's now defined in a file to avoid recompiles !
  QString line;
  if (file.open(QIODevice::ReadOnly) ) {       
    QTextStream t( &file );        // use a text stream
    while ( !t.atEnd() ) {           
      line = t.readLine();         // line of text excluding '\n'
      //locationBox->addItem("10.0.0.21");
      locationBox->addItem(line);

    }
    // Close the file
    file.close();
  }

  
  connect (initialButton, SIGNAL(clicked()), SLOT(setInitial()));
  connect (readyButton, SIGNAL(clicked()), SLOT(setReady()));
  connect (setButton, SIGNAL(clicked()), SLOT(setSet()));
  connect (playingButton, SIGNAL(clicked()), SLOT(setPlaying()));
  connect (testingButton, SIGNAL(clicked()), SLOT(setTesting()));
  connect (penalisedButton, SIGNAL(clicked()), SLOT(setPenalised()));
  connect (finishedButton, SIGNAL(clicked()), SLOT(setFinished()));
  connect (topCamButton, SIGNAL(clicked()), SLOT(setTopCameraBehavior()));
  connect (botCamButton, SIGNAL(clicked()), SLOT(setBottomCameraBehavior()));
  connect (testOdometryButton, SIGNAL(clicked()), SLOT(setTestOdometry()));
  
  connect (restartLuaButton, SIGNAL(clicked()), parent, SLOT(remoteRestartLua()));
  connect (uploadButton, SIGNAL(clicked()), this, SLOT(sendLua()));
  connect (uploadButton, SIGNAL(clicked()), this, SLOT(sendPython()));
  connect (configButton, SIGNAL(clicked()), this, SLOT(sendSimpleConfig()));
  connect (verifyConfigButton, SIGNAL(clicked()), this, SLOT(verifySimpleConfig()));

  connect (downLogsButton, SIGNAL(clicked()), this, SLOT(getLogs()));
  connect (removeLogsButton, SIGNAL(clicked()), this, SLOT(removeLogs()));

  connect (upBinaryButton, SIGNAL(clicked()), this, SLOT(sendBinary()));
  connect (upAllButton, SIGNAL(clicked()), this, SLOT(sendAll()));
  connect (upEveryButton, SIGNAL(clicked()), this, SLOT(sendEverything()));

  connect (upColorButton, SIGNAL(clicked()), this, SLOT(sendColorTable()));
  connect (upAutoloadButton, SIGNAL(clicked()), this, SLOT(sendAutoloadFile()));
  connect (upVisionButton, SIGNAL(clicked()), this, SLOT(sendVision()));
  connect (upMotionButton, SIGNAL(clicked()), this, SLOT(sendMotion()));
  connect (upNaoButton, SIGNAL(clicked()), this, SLOT(sendInterface()));
  connect (upMofButton, SIGNAL(clicked()), this, SLOT(sendMotionFiles()));
  connect (upCfgButton, SIGNAL(clicked()), this, SLOT(sendConfigFiles()));

  connect (verifyEveryButton, SIGNAL(clicked()), this, SLOT(verifyEverything()));

  connect (restartNaoQiButton, SIGNAL(clicked()), this, SLOT(restartNaoQi()));
  connect (mp3Button, SIGNAL(clicked()), this, SLOT(sendMp3Files()));
  connect (locationBox, SIGNAL(currentIndexChanged(int)), this, SLOT(locationChanged(int)));
  connect (resetTopButton, SIGNAL(clicked()), this, SLOT(resetTopCamera()));
  connect (resetBottomButton, SIGNAL(clicked()), this, SLOT(resetBottomCamera()));

  connect (stopNaoqiButton, SIGNAL(clicked()), this, SLOT(stopNaoqi()));
  connect (startNaoqiButton, SIGNAL(clicked()), this, SLOT(startNaoqi()));

  locationBox->setCurrentIndex(0);
}

void FilesWindow::loadConfig(QTextStream &t) {
  bool ok;
  int prevIndex = UTMainWnd::loadInt(t,ok);
  if (ok && (prevIndex < locationBox->count())) {
    locationBox->setCurrentIndex(prevIndex);
    locationChanged(-1);
  }
}

void FilesWindow::saveConfig(QTextStream &t) {
  // save the index
  t << locationBox->currentIndex() << "\n";
}

void FilesWindow::setInitial() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "SI");
}

void FilesWindow::setReady() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "SR");
}

void FilesWindow::setSet() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "SS");
}

void FilesWindow::setPlaying() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "SP");
}

void FilesWindow::setPenalised() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "SX");
}

void FilesWindow::setFinished() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "SF");
}

void FilesWindow::setTesting() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "ST");
}

void FilesWindow::setBottomCameraBehavior() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "SCB");
}

void FilesWindow::setTopCameraBehavior() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "SCT");
}

void FilesWindow::setTestOdometry() {
  QString cmd = "SO ";
  cmd += QString::number(odom_fwd->value()) + " " + QString::number(odom_side->value()) + " " + QString::number(odom_turn->value()) + " " + QString::number(odom_time->value()) + " ";
  std::cout << "sending test odometry: " << cmd.toStdString() << std::endl;
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), cmd);
}

void FilesWindow::resetTopCamera() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "CRT");
}

void FilesWindow::resetBottomCamera() {
  ((UTMainWnd*)parent)->sendUDPCommand(locationBox->currentText(), "CRB");
}

void FilesWindow::stopNaoqi() {
  naoqiCommand("stop");
}

void FilesWindow::startNaoqi() {
  naoqiCommand("start");
}

bool FilesWindow::sendCopyRobotCommand(QString command, bool verbose) {

  QString address = ((UTMainWnd*)parent)->getCurrentAddress();
  QProcess copy_robot;

  QString cmd(basePath);
  cmd += ("bin/copy_robot ");
  cmd += address + " ";
  cmd += command;
  if (verbose)
    std::cout << "Executing: " << cmd.toStdString() << std::endl;

  copy_robot.start(cmd);
  if (!copy_robot.waitForStarted()) {
    cout << "Unable to launch copy robot script" << endl << flush;
    return false;
  }
  copy_robot.closeWriteChannel();
  copy_robot.waitForFinished();

  if (verbose) { // print stdout and stderr as well
    QByteArray out_result = copy_robot.readAll();
    QString outStr(out_result);
    std::cout << outStr.toStdString();
  } else { // only print stderr
    QByteArray err_result = copy_robot.readAllStandardError();
    QString resultStr(err_result);
    std::cout << resultStr.toStdString();
  }

  // TODO: the exist status always seems 0, hence this does not really work
  bool returnState = copy_robot.exitStatus() == QProcess::NormalExit;

  if (returnState)
    filesStatus->showMessage(command + " succeeded for " + address);
  else
    filesStatus->showMessage(command + " failed for " + address);

  return returnState;
}

void FilesWindow::sendLua(bool verbose) {
  std::cout << "-- Sending lua --" << std::endl;
  sendCopyRobotCommand("lua",verbose);
}

void FilesWindow::verifyLua(bool verbose) {
  std::cout << "-- Verifying lua --" << std::endl;
  sendCopyRobotCommand("lua --verify",verbose);
}

void FilesWindow::sendPython(bool verbose) {
  std::cout << "-- Sending lua --" << std::endl;
  sendCopyRobotCommand("python",verbose);
}

void FilesWindow::verifyPython(bool verbose) {
  std::cout << "-- Verifying lua --" << std::endl;
  sendCopyRobotCommand("python --verify",verbose);
}

void FilesWindow::sendBinary(bool verbose) {
  std::cout << "-- Sending binaries --" << std::endl;
  sendCopyRobotCommand("nao motion vision",verbose);
}

void FilesWindow::verifyBinary(bool verbose) {
  std::cout << "-- Verifying binaries --" << std::endl;
  sendCopyRobotCommand("nao motion vision --verify",verbose);
}

void FilesWindow::sendAll(bool verbose) {
  std::cout << "-- Sending all --" << std::endl;
  sendCopyRobotCommand("all",verbose);
}

void FilesWindow::verifyAll(bool verbose) {
  std::cout << "-- Verify all --" << std::endl;
  sendCopyRobotCommand("all --verify",verbose);
}

void FilesWindow::sendEverything(bool verbose) {
  std::cout << "-- Sending everything --" << std::endl;
  sendCopyRobotCommand("everything",verbose);
}

void FilesWindow::verifyEverything(bool verbose) {
  std::cout << "-- Verify everything --" << std::endl;
  sendCopyRobotCommand("everything --verify",verbose);
  verifySimpleConfig();
}

void FilesWindow::sendVision(bool verbose) {
  std::cout << "-- Sending vision --" << std::endl;
  sendCopyRobotCommand("vision",verbose);
}

void FilesWindow::verifyVision(bool verbose) {
  std::cout << "-- Verifying vision --" << std::endl;
  sendCopyRobotCommand("vision --verify",verbose);
}

void FilesWindow::sendMotion(bool verbose) {
  std::cout << "-- Sending motion --" << std::endl;
  sendCopyRobotCommand("motion",verbose);
}

void FilesWindow::verifyMotion(bool verbose) {
  std::cout << "-- Verifying motion --" << std::endl;
  sendCopyRobotCommand("motion --verify",verbose);
}

void FilesWindow::sendInterface(bool verbose) {
  std::cout << "-- Sending nao interface --" << std::endl;
  sendCopyRobotCommand("nao",verbose);
}

void FilesWindow::verifyInterface(bool verbose) {
  std::cout << "-- Verifying nao interface --" << std::endl;
  sendCopyRobotCommand("nao --verify",verbose);
}

void FilesWindow::sendMotionFiles(bool verbose) {
  std::cout << "-- Sending motion files --" << std::endl;
  sendCopyRobotCommand("motion_file",verbose);
}

void FilesWindow::verifyMotionFiles(bool verbose) {
  std::cout << "-- Verifying motion files --" << std::endl;
  sendCopyRobotCommand("motion_file --verify",verbose);
}

void FilesWindow::sendColorTable(bool verbose) {
  std::cout << "-- Sending color tables --" << std::endl;
  sendCopyRobotCommand("color_table",verbose);
}

void FilesWindow::verifyColorTable(bool verbose) {
  std::cout << "-- Verifying color tables --" << std::endl;
  sendCopyRobotCommand("color_table --verify",verbose);
}

void FilesWindow::sendWireless(bool verbose) {
  std::cout << "-- Sending wireless --" << std::endl;
  sendCopyRobotCommand("wireless",verbose);
}

//void FilesWindow::verifyWireless(bool verbose) {
  //std::cout << "-- Verifying wireless --" << std::endl;
  //sendCopyRobotCommand("wireless --verify",verbose);
//}

void FilesWindow::sendAutoloadFile(bool verbose) {
  std::cout << "-- Sending autoload --" << std::endl;
  sendCopyRobotCommand("autoload",verbose);
}

void FilesWindow::verifyAutoloadFile(bool verbose) {
  std::cout << "-- Verifying autoload --" << std::endl;
  sendCopyRobotCommand("autoload --verify",verbose);
}

void FilesWindow::sendConfigFiles(bool verbose) {
  std::cout << "-- Sending config files --" << std::endl;
  sendCopyRobotCommand("config_file",verbose);
  sendCopyRobotCommand("scripts",verbose);
}

void FilesWindow::verifyConfigFiles(bool verbose) {
  std::cout << "-- Verifying config files --" << std::endl;
  sendCopyRobotCommand("config_file --verify",verbose);
  sendCopyRobotCommand("scripts --verify",verbose);
}

void FilesWindow::sendSimpleConfig(bool verbose) {
  std::cout << "-- Sending config.txt --" << std::endl;
  sendCopyRobotCommand("simple_config --team " + QString::number(teamNumBox->value()) + " --role " + QString::number(roleBox->value()),verbose);
}

void FilesWindow::verifySimpleConfig(bool verbose) {
  std::cout << "-- Verifying config.txt --" << std::endl;
  sendCopyRobotCommand("simple_config --team " + QString::number(teamNumBox->value()) + " --role " + QString::number(roleBox->value()) + " --verify",verbose);
}

void FilesWindow::sendFile(QString from, QString to, QString name){
  QString address = ((UTMainWnd*)parent)->getCurrentAddress();
  // make sure it is a robot
  if (address == "localhost") {
    cout << "Error, attempt to copy to localhost" << endl;
    filesStatus->showMessage("Error, attempt to copy to localhost");
    return;
  }

  cout << "Copying " << name.toStdString() << " to " << address.toStdString() << " ... " << flush;
  filesStatus->showMessage("Copying "+name+ " to " + address);
  
  QProcess *scp;
  scp = new QProcess( this ); // memory allocation from heap, created with parent
  
  QString out = "nao@";
  out.append(address + ":" + to);
  QStringList cmd;
  cmd.push_back(QString("-avz"));
  cmd.push_back(from);
  cmd.push_back(out);
  scp->start("rsync", cmd);
  if (!scp->waitForStarted()) {
    cout << "sendLib Error 1" << endl << flush;
    return;
  }
  std::cout << cmd.join(" ").toStdString() << std::endl;

  scp->closeWriteChannel();

  scp->waitForFinished();

  QByteArray result = scp->readAll();
  cout << "Done!\n" << endl;
  filesStatus->showMessage(name +" copied to " + address);
}  

void FilesWindow::sendMp3Files() {
  
  sendFile(dataPath + "fight.mp3","~/data","fight.mp3");
  sendFile(dataPath + "eyes.mp3","~/data","eyes.mp3");

}

void FilesWindow::naoqiCommand(QString c) {
  QString address = ((UTMainWnd*)parent)->getCurrentAddress();
  // make sure it is a robot
  if (address == "localhost") {
    cout << ("Error, attempt to " + c + " localhost").toStdString() << endl;
    filesStatus->showMessage("Error, attempt to " + c + " localhost");
    return;
  }
  
  QStringList lst;

  cout << (c + " NaoQi on " + address).toStdString() << endl;
  filesStatus->showMessage(c + "ing NaoQi on " + address);
  QProcess *scp;
  scp = new QProcess( this ); // memory allocation from heap, created with parent

  QString out = "nao@";
  out.append(address);
  QStringList cmd;
  cmd.push_back(out);
  //cmd.push_back("/etc/init.d/naoqi " + c);
  cmd.push_back("sudo /etc/init.d/naoqi " + c + " &> /dev/null");
  scp->start("ssh", cmd);
  if (!scp->waitForStarted()) {
    cout << (c + " Error 1").toStdString() << endl << flush;
    return;
  }
  scp->waitForFinished(120000);
  scp->closeWriteChannel();
  QByteArray result = scp->readAll();
  cout << "Done!\n" << endl;
  filesStatus->showMessage("NaoQi " + c + "ed on " + address);
}

void FilesWindow::restartNaoQi() {
  naoqiCommand("restart");
}


void FilesWindow::getLogs() {

  QString address = ((UTMainWnd*)parent)->getCurrentAddress();
  // make sure it is a robot
  if (address == "localhost") {
    cout << "Error, attempt to copy to localhost" << endl;
    filesStatus->showMessage("Error, attempt to copy to localhost");
    return;
  }

  // First get logs from robot
  QStringList lst;
  cout << "Copying logs from " << address.toStdString() << "\n";
  filesStatus->showMessage("Copying logs from " + QString(address));
  QProcess *scp;
  scp = new QProcess( this ); // memory allocation from heap, created with parent
  scp->setProcessChannelMode(QProcess::ForwardedChannels); // Forward stdout/stderr to main

  QString out = "nao@";
  out.append(address);
  out.append(":~/logs/vision*");

  QStringList cmd;
  cmd.push_back("-avz");
  cmd.push_back(out);
  cmd.push_back(logPath);
  scp->start("rsync", cmd);

  if (!scp->waitForStarted()) {
    cout << "Unable to get logs" << endl << flush;
    delete scp;
    return;
  }

  scp->waitForFinished(-1);
  scp->closeWriteChannel();
  delete scp;

  cout << "Done!" << endl;
  filesStatus->showMessage("Logs copied from " + QString(address));
}

void FilesWindow::removeLogs(){

  QString address = ((UTMainWnd*)parent)->getCurrentAddress();
  // make sure it is a robot
  if (address == "localhost") {
    cout << "Error, attempt to copy to localhost" << endl;
    filesStatus->showMessage("Error, attempt to copy to localhost");
    return;
  }

  // Then remove logs from robot
  cout << "Removing logs from " << address.toStdString() << " ... " << flush;
  filesStatus->showMessage("Removing logs from " + QString(address));
  QProcess *ssh;
  ssh = new QProcess( this ); // memory allocation from heap, created with parent

  QString out = "nao@";
  out.append(address);

  QStringList cmd2;
  cmd2.push_back(out);
  cmd2.push_back("rm -rf ~/logs/vision*");
  ssh->start("ssh", cmd2);

  if (!ssh->waitForStarted()) {
    cout << "getLogs Error 2" << endl << flush;
    return;
  }

  ssh->closeWriteChannel();
  ssh->waitForFinished();

  cout << "Done!\n" << endl;
  filesStatus->showMessage("Logs removed from " + QString(address));

}


void FilesWindow::checkStatus(){
  // check status of robot
  QStringList lst;
  QProcess *fping;
  fping = new QProcess( this ); // memory allocation from heap, created with parent

  QString address = ((UTMainWnd*)parent)->getCurrentAddress();
  
  if (!address.contains("core")) {
    QStringList cmd;
    cmd.push_back("-r1 -t100");
    cmd.push_back(address);
    
    fping->start("fping", cmd);
    
    if (!fping->waitForStarted()) {
      cout << "fping Error 1 - Try installing fping, if that doesn't work then comment this out" << endl << flush;
      return;
    }
    
    fping->closeWriteChannel();
    fping->waitForFinished();
    
    QByteArray result = fping->readAll();
    
    // TODO: how to set color
    whichRobot->setText(address);
    robotStatus->setAutoFillBackground(true);
    if (result.contains("alive")) 
      robotStatus->setText("Alive");
    else
      robotStatus->setText("Dead");

  }

  // check every 10 seconds 
  statusTimer->singleShot(10 * 1000,this,SLOT(checkStatus()));
  
}

// enable or disable the buttons
void FilesWindow::enableButtons(bool b){

  restartLuaButton->setEnabled(b);
  uploadButton->setEnabled(b);
  configButton->setEnabled(b);

  downLogsButton->setEnabled(b);
  removeLogsButton->setEnabled(b);

  upBinaryButton->setEnabled(b);
  upAllButton->setEnabled(b);
  upEveryButton->setEnabled(b);
  upColorButton->setEnabled(b);
  upAutoloadButton->setEnabled(b);

  restartNaoQiButton->setEnabled(b);
  stopNaoqiButton->setEnabled(b);
  startNaoqiButton->setEnabled(b);

  upVisionButton->setEnabled(b);
  upMotionButton->setEnabled(b);
  upNaoButton->setEnabled(b);
  upMofButton->setEnabled(b);
  upCfgButton->setEnabled(b);

  verifyEveryButton->setEnabled(b);
  verifyConfigButton->setEnabled(b);

  /*
    mp3Button->setEnabled(b);
  */
}



// called from control window
void FilesWindow::changeLocationIndex(int index) {
  // change location index
  locationBox->setCurrentIndex(index);
}

void FilesWindow::locationChanged(int index){
  // update address/status
  QString address = ((UTMainWnd*)parent)->getCurrentAddress();
  whichRobot->setText(address);
  robotStatus->setText("Unknown");

  // update log select window
  ((UTMainWnd*)parent)->logSelectWnd_->updateSelectedIP(address);

  if (index >= 0)
    ((UTMainWnd*)parent)->saveConfig();
}

void FilesWindow::setCurrentLocation(QString ip) {
  int index = locationBox->findText(ip);
  if (index == -1){
    locationBox->insertItem(0, ip);
    index = 0;
  }
  locationBox->setCurrentIndex(index);
}
