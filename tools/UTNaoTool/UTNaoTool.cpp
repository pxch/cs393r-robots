#include <QApplication>
#include <iostream>

#include "UTMainWnd.h" 
#include "FilesWindow.h"
#include "BehaviorSimulation.h"
#include <lua/LuaModule.h>

namespace {

  bool displayMotionWindow = false;
  bool displayVisionWindow = false;
  bool displayFilesWindow = false;
  bool displayWorldWindow = false;
  bool displayPlotWindow = false;
  bool displayTeamWindow = false;
  bool openRecent = false;
  bool runCore = false;
  bool locOnly = false;
  bool visOnly = false;
  std::string logFile;
  int logindex = -1;
  QString IP;

  int startFrame = 0;
  int endFrame = -1;

  bool runHeadlessBehaviorSim = false;
  std::string headlessConfig = "";
}

void displayHelp() {
  std::cout << "\n\nOptions :\n";
  std::cout << "\t-o <file>\topens log file\n";
  std::cout << "\t-m\topens motion window\n";
  std::cout << "\t-v\topens vision window\n";
  std::cout << "\t-f\topens files window\n";
  //std::cout << "\t-c\topens control window\n";
  std::cout << "\t-w\topens world window\n";
  //std::cout << "\t-l\topens log window\n";
  //std::cout << "\t-v\topens vision window\n";
  //std::cout << "\t-k\topens kinematics window\n";
  std::cout << "\t-z\tdefaults to run core\n";
  std::cout << "\t-r\topens most recent log\n";
  std::cout << "\t-t\topens team cfg window\n";
  std::cout << "\t-s\tselect start frame\n";
  std::cout << "\t-e\tselect end frame\n";
  std::cout << "\t-i <0-9>\tinitializes with IP set to 192.168.25.9x\n";
  std::cout << "\t-c <0-9>\tinitializes with IP set to 11.0.1.9x\n";
  std::cout << "\t-l\tenables localization only\n";
  std::cout << "\t-^\tenables vision only\n";
  //std::cout << "\t-n\tdo NOT run core (for slow machines)\n";
  std::cout << "\t-h -?\t display this help\n\n";
}

int getParameters(int argc, char **argv) {
  char ch;
  const char* optflags = "mpvfwrtb:o:i:c:s:e:h?:zl^";
  while(-1 != (ch = getopt(argc, argv, optflags))) {
    switch(ch) {
    case 'm':
      displayMotionWindow = true;
      break;
    case 'p':
      displayPlotWindow = true;
      break;
    case 'v':
      displayVisionWindow = true;
      break;
    case 'f':
      displayFilesWindow = true;
      break;
    case 'w':
      displayWorldWindow = true;
      break;
    case 't':
      displayTeamWindow = true;
      break;
    case 'b':
      runHeadlessBehaviorSim = true;
      headlessConfig = std::string(optarg);
      break;
    case 'o':
      logFile = std::string(optarg);
      break;
    case 'r':
      openRecent = true;
      break;
    case 'i':
      IP = "192.168.25.9"+QString(optarg);
      break;
    case 'c':
      IP = "11.0.1.9"+QString(optarg);
      break;
    case 's':
      startFrame = atoi(optarg);
      break;
    case 'e':
      endFrame = atoi(optarg);
      break;
    case 'z':
      runCore = true;
      break;
    case 'l':
      locOnly = true;
      break;
    case '^':
      visOnly = true;
      break;
    case 'h':
    case '?':
      displayHelp();
      return 0;
    }
  }
  return 1;
}

int main(int argc, char **argv) {

  if (!getParameters(argc, argv))
    return 0;

  if (runHeadlessBehaviorSim) {
    std::string cmd = std::string("roleSwitch.setupRoles('") + headlessConfig + "')";
    BehaviorSimulation behaviorSim(WO_OPPONENT_LAST,false,false);
    for (int i = 1; i <= WO_OPPONENT_LAST; i++) {
      if (behaviorSim.sims[i] != NULL)
        behaviorSim.sims[i]->core->lua_->call(cmd);
    }
    if ((((float)rand())/((float)RAND_MAX)) < 0.5) {
      behaviorSim.changeSimulationKickoff();
    }
    while (behaviorSim.numHalves < 1) {
      behaviorSim.simulationStep();
      //if (((int)behaviorSim.halfTimer % 50) == 0)
        //std::cout << behaviorSim.halfTimer << std::endl;
    }
    std::cout << "Score: " << behaviorSim.simBlueScore << " " << behaviorSim.simRedScore << std::endl;
    return 0;
  }

  QApplication a(argc, argv);

  UTMainWnd* main;
  if (logFile.empty()) {
    main = new UTMainWnd(NULL);
  } else {
    main = new UTMainWnd(logFile.c_str());
  }

  main->setFrameRange(startFrame, endFrame);

  if (IP.size() > 0){
    std::cout << "IP : " << IP.toStdString() << std::endl;
    main->filesWnd_->setCurrentLocation(IP);
  }

  if (openRecent) main->openRecent();

  main->show();
  
  if (displayMotionWindow)
    main->openMotionWnd();
  if (displayPlotWindow)
    main->openPlotWnd();
  if (displayVisionWindow)
    main->openVisionWnd();
  if (displayFilesWindow)
    main->openFilesWnd(); 
  if (displayWorldWindow)
    main->openWorldWnd();
  if (displayTeamWindow)
    main->openTeamWnd();
  if (locOnly)
    main->localizationOnlyCheckBox->setCheckState(Qt::Checked);
  if (visOnly)
    main->visionOnlyCheckBox->setCheckState(Qt::Checked);
  if(runCore)
    main->setCore(true);
  if(logindex >= 0)
    main->gotoSnapshot(logindex);

  a.exec();

  delete main;
  return 0;
}
