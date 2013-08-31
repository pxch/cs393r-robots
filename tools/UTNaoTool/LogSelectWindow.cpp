#include <QtGui>
#include "LogSelectWindow.h"
#include <iostream>
#include "UTMainWnd.h"

using namespace std;

unsigned int MAX_MODULES_PER_COLUMN = 15;

LogSelectWindow::LogSelectWindow(QMainWindow* pa, std::vector<std::string> &block_names) : QWidget(), naoUDP(NULL) {
  QGridLayout *layout = new QGridLayout;
  parent = pa;

  //Snapshot* s = new Snapshot();
  //NUM_MODULES = s->memModules.size();
  // add behavior_trace
  std::ifstream in((std::string(getenv("NAO_HOME")) + "/data/moduleList.txt").c_str());
  std::string block_name;
  while (in.good()) {
    in >> block_name;
    if (in.good())
      block_names.push_back(block_name);
  }
  block_names.push_back(std::string("behavior_trace"));

  NUM_MODULES = block_names.size();

  // module names
  QString moduleNames[NUM_MODULES];

  for (int i = 0; i < NUM_MODULES; i++)
    moduleNames[i] = QString::fromStdString(block_names[i]);


  //QLabel* batchLabel = new QLabel("Batch");

  moduleLabels = new QLabel[NUM_MODULES];
  moduleChecks = new QCheckBox[NUM_MODULES];

  // set values, add to layout
  int column = 0;
  int row = 0;
  for (int i = 0; i < NUM_MODULES; i++) {
    if (row == 0) {
      QLabel* moduleLabel = new QLabel("Module");
      QLabel* checkLabel = new QLabel("Logged?");
      moduleLabel->setFont( QFont( "Arial", 10, QFont::Bold ) );
      checkLabel->setFont( QFont( "Arial", 10, QFont::Bold ) );

      layout->addWidget(moduleLabel,row,column);
      layout->addWidget(checkLabel,row,column+1);
      row++;
    }
    moduleLabels[i].setText(moduleNames[i]);

    if (moduleNames[i] == "vision_frame_info") {
      moduleChecks[i].setChecked(true);
      moduleChecks[i].setDisabled(true);
    }
    if (moduleNames[i] == "robot_state") {
      moduleChecks[i].setChecked(true);
      moduleChecks[i].setDisabled(true);
    }
    if (moduleNames[i] == "robot_info") {
      moduleChecks[i].setChecked(true);
      moduleChecks[i].setDisabled(true);
    }

    // add to layout
    layout->addWidget(&moduleLabels[i], row, column);
    layout->addWidget(&moduleChecks[i], row, column+1);

    row++;
    if ((uint16_t)(row - 1) == MAX_MODULES_PER_COLUMN) {
      row = 0;
      column += 2;
    }
  }

  sendButton = new QPushButton("Send");
  logButton = new QPushButton("Logging is OFF");
  forceStopLogButton = new QPushButton("Force stop logging");
  batchButton = new QCheckBox();
  frameCount = new QSpinBox();
  frameCount->setMaximum(500);
  frameCount->setValue(0);
  log_enabled_ = false;

  frequency = new QDoubleSpinBox();
  frequency->setMaximum(30.0);
  frequency->setMinimum(0.0);
  frequency->setValue(0.0);
  frequency->setSingleStep(0.5);
  QLabel* freqLabel = new QLabel("Frequency");
  freqLabel->setText("Log Frequency (s)");


  layout->addWidget(sendButton, NUM_MODULES+2, 0);
  layout->addWidget(logButton, NUM_MODULES+3, 0);
  layout->addWidget(frameCount, NUM_MODULES+3, 1);
  layout->addWidget(forceStopLogButton, NUM_MODULES+3, 2);
  layout->addWidget(frequency, NUM_MODULES + 4, 1);
  layout->addWidget(freqLabel, NUM_MODULES + 4, 0);

  //layout->addWidget(batchButton, NUM_MODULES+4, 1); // no batch yet
  //layout->addWidget(batchLabel, NUM_MODULES+4, 0);

  // Todd: add some 'group selection' buttons
  int NUM_GROUPS = 5;
  groupLabels = new QLabel[NUM_GROUPS];
  groupChecks = new QCheckBox[NUM_GROUPS];

  int ind = 0;
  groupLabels[ind++].setText("Localization");
  groupLabels[ind++].setText("Vision");
  groupLabels[ind++].setText("Vision w/ Raw");
  groupLabels[ind++].setText("Behavior");
  groupLabels[ind++].setText("ALL");

  for (int i = 0; i < NUM_GROUPS; i++){
    layout->addWidget(&groupLabels[i], NUM_MODULES+5+i, 0);
    layout->addWidget(&groupChecks[i], NUM_MODULES+5+i, 1);
  }

  connect (logButton, SIGNAL(clicked()), this, SLOT(toggleLogEnabled()));
  connect (forceStopLogButton, SIGNAL(clicked()), this, SLOT(stopLog()));
  //connect (batchButton, SIGNAL(toggled(bool)), parent, SLOT(setBatchEnabled(bool)));
  connect (sendButton, SIGNAL(clicked()), this, SLOT(sendLogSettings()));

  ind = 0;
  connect(&groupChecks[ind++], SIGNAL(toggled(bool)), this, SLOT(locGroupToggled(bool)));
  connect(&groupChecks[ind++], SIGNAL(toggled(bool)), this, SLOT(visionGroupToggled(bool)));
  connect(&groupChecks[ind++], SIGNAL(toggled(bool)), this, SLOT(visionRawGroupToggled(bool)));
  connect(&groupChecks[ind++], SIGNAL(toggled(bool)), this, SLOT(behaviorGroupToggled(bool)));
  connect(&groupChecks[ind++], SIGNAL(toggled(bool)), this, SLOT(allGroupToggled(bool)));


  setLayout(layout);

  resize(120,200);

  setWindowTitle(tr("Select Modules to Log"));
}


void LogSelectWindow::sendLogSettings(){

  cout << "Send log settings" << endl;

  QString cmd = "LS";

  //std::vector<bool> en;
  //en.resize(NUM_MODULES, false);

  for (int i = 0; i < NUM_MODULES; i++){
    cmd += moduleLabels[i].text();
    if (moduleChecks[i].isChecked())
      cmd += " 1";
    else
      cmd += " 0";
    cmd += ",";
  }
  cmd += '|';
  // send this vector
  //((UTMainWnd*)parent)->sendModuleSelections(en, NUM_MODULES);
  ((UTMainWnd*)parent)->sendUDPCommand(((UTMainWnd*)parent)->getCurrentAddress(),cmd);

  cout << "Selections sent" << endl;

}

void LogSelectWindow::toggleLogEnabled() {
  if (log_enabled_) {
    stopLog();
  } else {
    logModeOn();
    QString cmd = "LB,";
    cmd += QString::number(frameCount->value());
    cmd += ",";
    cmd += QString::number(frequency->value());
    cmd += ",";
    sendLogMessage(cmd);
  }
}

void LogSelectWindow::stopLog() {
  logButton->setEnabled(false);
  QString cmd = "LE";
  sendLogMessage(cmd);
}

void LogSelectWindow::sendLogMessage(const QString &msg) {
  ((UTMainWnd*)parent)->sendUDPCommand(((UTMainWnd*)parent)->getCurrentAddress(),msg);
  listenForLoggingStatus();
}

void LogSelectWindow::logModeOn() {
    log_enabled_ = true;
    logButton->setText("Logging is ON");
}

void LogSelectWindow::logModeOff() {
    log_enabled_ = false;
    logButton->setText("Logging is OFF");
    logButton->setEnabled(true);
}

void LogSelectWindow::listenForLoggingStatus() {
    QString address = ((UTMainWnd*)parent)->getCurrentAddress();
    if (naoUDP != NULL)
      delete naoUDP;
    naoUDP = new UDPWrapper(TOOL_UDP_PORT, false, address.toStdString().c_str());
    naoUDP->startListenThread(&(LogSelectWindow::listenUDP),this);
}

void* LogSelectWindow::listenUDP(void* arg) {
    LogSelectWindow* window = reinterpret_cast<LogSelectWindow*>(arg);
    char buffer[1024];
    sleep(0.5);
    window->naoUDP->recv(buffer,sizeof(buffer));
    std::string s(buffer);
    if(s[0] == 'L' && s[1] == 'E')
        window->logModeOff();
    return 0;
}

void LogSelectWindow::locGroupToggled(bool toggle){
  for (int i = 0; i < NUM_MODULES; i++){
    if (moduleLabels[i].text() == "game_state" ||
        moduleLabels[i].text() == "localization" ||
        //moduleLabels[i].text() == "opponents" ||
        //moduleLabels[i].text() == "robot_vision" ||
        moduleLabels[i].text() == "team_packets" ||
        moduleLabels[i].text() == "vision_odometry" ||
        moduleLabels[i].text() == "vision_joint_angles" ||
        moduleLabels[i].text() == "world_objects")
    moduleChecks[i].setChecked(toggle);
  }
}

void LogSelectWindow::visionGroupToggled(bool toggle){
  for (int i = 0; i < NUM_MODULES; i++){
    if (moduleLabels[i].text() == "robot_vision" ||
        moduleLabels[i].text() == "camera_info" ||
        moduleLabels[i].text() == "world_objects" ||
        moduleLabels[i].text() == "vision_body_model" ||
        moduleLabels[i].text() == "vision_joint_angles" ||
        moduleLabels[i].text() == "vision_sensors" ||
        moduleLabels[i].text() == "game_state"
        )
    moduleChecks[i].setChecked(toggle);
  }
}

void LogSelectWindow::visionRawGroupToggled(bool toggle){
  for (int i = 0; i < NUM_MODULES; i++){
    if (moduleLabels[i].text() == "robot_vision" ||
        moduleLabels[i].text() == "camera_info" ||
        moduleLabels[i].text() == "world_objects" ||
        moduleLabels[i].text() == "vision_body_model" ||
        moduleLabels[i].text() == "vision_joint_angles" ||
        moduleLabels[i].text() == "vision_sensors" ||
        moduleLabels[i].text() == "raw_image" ||
        moduleLabels[i].text() == "game_state"
        )
    moduleChecks[i].setChecked(toggle);
  }
}

void LogSelectWindow::behaviorGroupToggled(bool toggle){
  for (int i = 0; i < NUM_MODULES; i++){
    if (moduleLabels[i].text() == "game_state" ||
        moduleLabels[i].text() == "localization" ||
        //moduleLabels[i].text() == "opponents" ||
        //moduleLabels[i].text() == "robot_vision" ||
        moduleLabels[i].text() == "team_packets" ||
        moduleLabels[i].text() == "vision_odometry" ||
        moduleLabels[i].text() == "vision_joint_angles" ||
        moduleLabels[i].text() == "behavior" ||
        moduleLabels[i].text() == "vision_walk_request" ||
        moduleLabels[i].text() == "world_objects")
    moduleChecks[i].setChecked(toggle);
  }
}

void LogSelectWindow::allGroupToggled(bool toggle){
  for (int i = 0; i < NUM_MODULES; i++){
    if (moduleLabels[i].text() == "vision_frame_info") continue;
    if (moduleLabels[i].text() == "robot_state") continue;
    moduleChecks[i].setChecked(toggle);
  }
}

void LogSelectWindow::updateSelectedIP(QString address){
  sendButton->setText("Send to "+address);
}
