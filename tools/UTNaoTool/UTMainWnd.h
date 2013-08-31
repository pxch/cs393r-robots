#ifndef UTMAINWND_H
#define UTMAINWND_H

#include <QtGui/qmainwindow.h>
#include <vector>
#include <stdio.h>

#include <memory/Memory.h>
#include <memory/Log.h>
#include <communications/StreamingMessage.h>
#include <localization/LocalizationModule.h>

#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::tcp;
typedef boost::shared_ptr<tcp::socket> socket_ptr;


#include <QFileDialog>

#include <VisionCore.h>

#include "ui_MainWindow.h"

class UTMainWnd;
class FilesWindow;
class LogEditorWindow;
class LogSelectWindow;
class LogWindow;
class MotionWindow;
class PlotWindow;
class VisionWindow;
class WorldWindow;
class JointsWindow;
class WalkWindow;
class StateWindow;
class CameraWindow;
class SensorWindow;
class TeamConfigWindow;

void server(UTMainWnd *main);
void session(UTMainWnd *main,socket_ptr sock);

class UTMainWnd : public QMainWindow, public Ui_UTNaoTool {
Q_OBJECT
public:
  UTMainWnd(const char *filename = NULL, bool core = false);
  ~UTMainWnd();

  void sendUDPCommand(QString address, QString cmd);
  void sendUDPCommandToCurrent(QString cmd);
  QString getCurrentAddress();

  void setFrameRange(int start, int end) {
    startFrame = start;
    endFrame = end;
  }

  void loadConfig();
  void saveConfig();
  static int loadInt(QTextStream &t, bool &ok);

private:
  void loadLog(const char *filename);
  std::string logFile;

  VisionCore* visionCore_;

  std::vector<Memory> core_log_;
  Log* memory_log_;
  bool isStreaming_;
  bool runningCore_;
  bool coreAvailable_;

  Memory* memory_;
  uint16_t current_index_;

  int startFrame;
  int endFrame;

  // streaming
public:
  void processStream(socket_ptr sock);
  boost::asio::io_service io_service;
  tcp::acceptor acceptor;
  //QTcpServer tcpServer;
  //QTcpSocket *client;
  Memory stream_memory_;
  StreamingMessage stream_msg_;

  // windows
  FilesWindow* filesWnd_;
  LogEditorWindow* logEditorWnd_;
  LogSelectWindow* logSelectWnd_;
  LogWindow* logWnd_;
  MotionWindow* motionWnd_;
  PlotWindow* plotWnd_;
  VisionWindow* visionWnd_;
  WorldWindow* worldWnd_;
  JointsWindow* jointsWnd_;
  WalkWindow* walkWnd_;
  StateWindow* stateWnd_;
  CameraWindow* cameraWnd_;
  SensorWindow* sensorWnd_;
  TeamConfigWindow* teamWnd_;

  QString rcPath;

private:

public slots:
  void gotoSnapshot(int index);
  void nextSnapshot();
  void prevSnapshot();
  void play();
  void pause();

  bool openLog();
  void runCore();
  void rerunCore();
  void runLog();
  void setCore(bool value);
  void runStream();
  void stopStream();

  bool openRecent();

  void openFilesWnd();
  void openTeamWnd();
  void openLogEditorWnd();
  void openLogSelectWnd();
  void openLogWnd();
  void openMotionWnd();
  void openPlotWnd();
  void openVisionWnd();
  void openWorldWnd();
  void openJointsWnd();
  void openWalkWnd();
  void openSensorWnd();
  void openStateWnd();
  void openCameraWnd();

  void handleStreamFrame();

  void remoteRestartLua();
  void updateConfigFile();
  void updateCalibrationFile();
  void readCalibrationFile();

  //void tcpConnection();
  //void tcpReadyRead();
Q_SIGNALS:
  void newStreamFrame();
  void newLogFrame(int);
  void newLogLoaded(Log*);
  void runningCore();
  void setStreaming(bool);
};
#endif
