// This now works by setting the appropriate IP in the control window and then calling the Files window methods
// for scp'ing files


#include <QtGui>
#include "TeamConfigWindow.h"
#include "UTMainWnd.h"

#include "FilesWindow.h"

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <common/WorldObject.h>

using namespace std;

TeamConfigWindow::TeamConfigWindow(QMainWindow* p) {
  parent = p;

  setupUi(this);
  setWindowTitle(tr("Team Config"));
  connect (reloadConfigButton, SIGNAL(clicked()), this, SLOT(reloadLocalConfig()));
  connect (saveConfigButton, SIGNAL(clicked()), this, SLOT(saveLocalConfig()));

  connect (stopNaoQiButton, SIGNAL(clicked()), this, SLOT(stopNaoQi()));
  connect (startNaoQiButton, SIGNAL(clicked()), this, SLOT(startNaoQi()));
  connect (restartLuaButton, SIGNAL(clicked()), this, SLOT(restartLua()));
  connect (uploadEverythingButton, SIGNAL(clicked()), this, SLOT(uploadEverything()));
  connect (uploadBinaryButton, SIGNAL(clicked()), this, SLOT(uploadBinary()));
  connect (upLoadConfigButton, SIGNAL(clicked()), this, SLOT(uploadConfig()));
  connect (uploadColorButton, SIGNAL(clicked()), this, SLOT(uploadColor()));
  connect (uploadWirelessButton, SIGNAL(clicked()), this, SLOT(uploadWireless()));
  connect (uploadLuaButton, SIGNAL(clicked()), this, SLOT(uploadLua()));
  connect (uploadTimeButton, SIGNAL(clicked()), this, SLOT(uploadTime()));

  // set to our rc 2013 team # (1)
  GCNum->setValue(1);

  // check on robot's status
  // start timer for status updates
  QTimer *timer = new QTimer(this);
  connect (timer, SIGNAL(timeout()), this, SLOT(checkStatus()));
  // check on robots using fping every 15 seconds
  timer->start(15000);
}

void TeamConfigWindow::reloadLocalConfig() {
  QString teamConfigPath = QString(getenv("NAO_HOME")) + "/data/teamConfig.txt";

  localConfig = fopen(teamConfigPath.toAscii(),"rb"); // List of team
  if (localConfig == NULL) {
    cout << "Team Manager Window: No config to load" << std::endl;
    return;
  }
  rewind(localConfig);
  QTextStream in(localConfig);
  commonIP->setText(in.readLine());
  IP1->setText(in.readLine());
  IP2->setText(in.readLine());
  IP3->setText(in.readLine());
  IP4->setText(in.readLine());
  IP5->setText(in.readLine());
  GCNum->setValue(in.readLine().toInt());
  cout << "Team Manager Window: Config Loaded\n";
  fclose(localConfig);
}


void TeamConfigWindow::saveLocalConfig() {
  QString teamConfigPath = QString(getenv("NAO_HOME")) + "/data/teamConfig.txt";
  QFile file(teamConfigPath);
  if (file.open(QIODevice::WriteOnly) ) {
    QTextStream out(&file);
    out << commonIP->text() << "\n" << IP1->text() << "\n" << IP2->text() << "\n" << IP3->text() << "\n" << IP4->text() << "\n" << IP5->text() << "\n" << GCNum->value() << "\n" ;
    cout << "Team Manager Window: Config Written\n";
    file.close();
  }
}


QStringList TeamConfigWindow::getUploadList() {
  QStringList list;
  if (IP1->text()!="" && IPBox1->isChecked()) list.append(IP1->text());
  if (IP2->text()!="" && IPBox2->isChecked()) list.append(IP2->text());
  if (IP3->text()!="" && IPBox3->isChecked()) list.append(IP3->text());
  if (IP4->text()!="" && IPBox4->isChecked()) list.append(IP4->text());
  if (IP5->text()!="" && IPBox5->isChecked()) list.append(IP5->text());
  return list;
}

void TeamConfigWindow::startNaoQi() {
  cout << "Team Manager Window: Starting NaoQi's\n";

  QStringList list=getUploadList();
  int size=list.size();
  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]);

    // set it in control window and call files restartNaoQi on it
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);
    ((UTMainWnd*)parent)->filesWnd_->startNaoqi();

  }
}

void TeamConfigWindow::stopNaoQi() {
  cout << "Team Manager Window: Stopping NaoQi's\n";

  QStringList list=getUploadList();
  int size=list.size();
  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]);

    // set it in control window and call files restartNaoQi on it
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);
    ((UTMainWnd*)parent)->filesWnd_->stopNaoqi();

  }
}

void TeamConfigWindow::restartLua() {
  cout << "Team Manager Window: Restarting Lua\n";

  QStringList list=getUploadList();
  int size=list.size();
  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]); //toLatin1().data();

    // set it in control window and call files restartLua on it
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);
    ((UTMainWnd*)parent)->remoteRestartLua();

  }
}



void TeamConfigWindow::uploadEverything() {
  cout << "Team Manager Window: Upload Everything\n";

  QStringList list=getUploadList();
  int size=list.size();
  QProcess sshProcesses[WO_TEAM_LAST-WO_TEAM_FIRST+1]; // might not use them all

  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]);
    QString cmd("ssh nao@" + ip);
    sshProcesses[i].start(cmd);
  }
  sleep(2);

  uploadBinary();
  uploadLua();
  uploadConfig();
  uploadColor();
  //uploadWireless();

  for (int i = 0; i < size; i++) {
    sshProcesses[i].write("exit\n");
    //sshProcesses[i].closeWriteChannel();
    sshProcesses[i].waitForFinished();
    //sshProcesses[i].close();
  }
  cout << "Team Manager Window: Everything uploaded\n";
}

void TeamConfigWindow::uploadBinary() {

  //cout << "Team Manager Window: Upload Binary\n";
  QStringList list=getUploadList();
  int size=list.size();
  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]); //toLatin1().data();


    // set it in control window and call upload binary on it
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->sendBinary(false);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->verifyBinary(false);
  }
}



void TeamConfigWindow::uploadConfig() {
  //cout << "Team Manager Window: Upload Config\n";
  QStringList list=getUploadList();
  int size=list.size();
  for (int i=0; i<size; i++) {
    QString ip = list[i]; //toLatin1().data();
    // make config file

    int role = 0;
    if (ip==IP1->text() && IPBox1->isChecked()) role=1;
    else if (ip==IP2->text() && IPBox2->isChecked()) role=2;
    else if (ip==IP3->text() && IPBox3->isChecked()) role=3;
    else if (ip==IP4->text() && IPBox4->isChecked()) role=4;
    else if (ip==IP5->text() && IPBox5->isChecked()) role=5;

    ip = getFullIP(ip);

    // set these in Files window
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);
    ((UTMainWnd*)parent)->filesWnd_->roleBox->setValue(role);
    ((UTMainWnd*)parent)->filesWnd_->teamNumBox->setValue(GCNum->value());

    // call FilesWnd method
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->sendSimpleConfig(false);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->verifySimpleConfig(false);

    // Also copy static configuration files
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->sendConfigFiles(false);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->verifyConfigFiles(false);

    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->sendMotionFiles(false);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->verifyMotionFiles(false);
  }
}

void TeamConfigWindow::uploadColor() {
  //cout << "Team Manager Window: Upload Color Table\n";
  QStringList list=getUploadList();
  int size=list.size();
  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]); //toLatin1().data();

    // set it in control window and call upload binary on it
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->sendColorTable(false);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->verifyColorTable(false);

  }
}

void TeamConfigWindow::uploadWireless() {
  cout << "Team Manager Window: Upload Wireless\n";
  QStringList list=getUploadList();
  int size=list.size();
  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]); //toLatin1().data();

    // set it in control window and call upload binary on it
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->sendWireless(false);
    //std::cout << ip.toStdString() << " ";
    //((UTMainWnd*)parent)->filesWnd_->verifyWireless(false);

  }
}

void TeamConfigWindow::uploadLua() {
  //cout << "Team Manager Window: Upload Lua\n";

  QStringList list=getUploadList();
  int size=list.size();
  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]); //toLatin1().data();

    // set it in control window and call upload binary on it
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);

    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->sendLua(false);
    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->verifyLua(false);

  }
}



void TeamConfigWindow::checkStatus(){

  // only if this window is active
  if (!this->isVisible())
    return;

  // check status of robot
  for (int i=0; i<5; i++) {

    QString ip = IP1->text();
    if (i == 0){
      if (!IPBox1->isChecked()){
        IP1_status->setText("-");
        continue;
      }
    }
    else if (i == 1) {
      ip = IP2->text();
      if (!IPBox2->isChecked()){
        IP2_status->setText("-");
        continue;
      }
    }
    else if (i == 2) {
      ip = IP3->text();
      if (!IPBox3->isChecked()){
        IP3_status->setText("-");
        continue;
      }
    }
     else if (i == 3) {
      ip = IP4->text();
      if (!IPBox4->isChecked()){
        IP4_status->setText("-");
        continue;
      }
    }
     else if (i == 4) {
      ip = IP5->text();
      if (!IPBox5->isChecked()){
        IP5_status->setText("-");
        continue;
      }
    }

    QStringList lst;
    QProcess *fping;
    fping = new QProcess( this ); // memory allocation from heap, created with parent
    QStringList cmd;
    cmd.push_back("-r1 -t100");
    cmd.push_back(getFullIP(ip));
    fping->start("fping", cmd);
    if (!fping->waitForStarted()) {
      cout << "fping Error 1 - Try installing fping, if that doesn't work then comment this out" << endl << flush;
      return;
    }
    fping->closeWriteChannel();
    fping->waitForFinished();
    QByteArray result = fping->readAll();

    // Set Color to green if alive, red if dead
    if (ip==IP1->text()){
      if (result.contains("alive"))
        IP1_status->setText("Up");
      else
        IP1_status->setText("X");
    }
    else if (ip==IP2->text()) {
      if (result.contains("alive"))
        IP2_status->setText("Up");
      else
        IP2_status->setText("X");
    }
    else if (ip==IP3->text()){
      if (result.contains("alive"))
        IP3_status->setText("Up");
      else
        IP3_status->setText("X");
    }
    else if (ip==IP4->text()){
      if (result.contains("alive"))
        IP4_status->setText("Up");
      else
        IP4_status->setText("X");
    }
    else if (ip==IP5->text()){
      if (result.contains("alive"))
        IP5_status->setText("Up");
      else
        IP5_status->setText("X");
    }
  }

}


void TeamConfigWindow::uploadTime() {
  QStringList list=getUploadList();
  int size=list.size();
  std::cout << "TIME" << std::endl;
  for (int i=0; i<size; i++) {
    QString ip = getFullIP(list[i]); //toLatin1().data();

    // set it in control window and call upload binary on it
    ((UTMainWnd*)parent)->filesWnd_->setCurrentLocation(ip);

    std::cout << ip.toStdString() << " ";
    ((UTMainWnd*)parent)->filesWnd_->sendCopyRobotCommand("time",true);
  }
}

QString TeamConfigWindow::getFullIP(const QString &suffix) {
  QString common = commonIP->text();
  if (!common.endsWith("."))
    common += ".";
  return common + suffix;
}

