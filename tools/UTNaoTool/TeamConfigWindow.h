#ifndef TEAMCONFIG_WINDOW_H
#define TEAMCONFIG_WINDOW_H

#include <QWidget>

#include "ui_TeamConfigWindow.h"

class QLabel;
class QWidget;
class QFile;

class TeamConfigWindow : public QMainWindow, public Ui_TeamConfigWindow {
 Q_OBJECT

  public:
  TeamConfigWindow(QMainWindow* par);
  QString getFullIP(const QString &suffix);

  
  FILE* localConfig;
  
  QStringList getUploadList();
  QMainWindow* parent;

  public slots:
    void reloadLocalConfig();
    void saveLocalConfig();
    void startNaoQi();  
    void stopNaoQi();
    void restartLua();  
    void uploadEverything();
    void uploadBinary();
    void uploadConfig();
    void uploadColor();
    void uploadWireless();
    void uploadLua();
    void checkStatus();
    void uploadTime();
};


#endif
