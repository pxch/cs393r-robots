#ifndef LOG_WINDOW_H
#define LOG_WINDOW_H

#include <vector>
#include <string>

#include <memory/Memory.h>
#include <memory/FrameInfoBlock.h>
#include <memory/TextLogger.h>

#include "ui_LogWindow.h"

class LogWindow : public QMainWindow, public Ui_UTLogWindow {
 Q_OBJECT

  public:
    LogWindow(QMainWindow* pa);

    void updateFrame(Memory* mem);
    void updateMemoryBlocks(Memory *mem);

    void loadTextFile(const char* filename);
    std::vector<std::string> textEntries;
    QMainWindow* parent;
    int prevFrame;
    int currFrame;
    
    void setText(std::vector<std::string> text);

    int moduleType;

    void loadConfig(QTextStream &t);
    void saveConfig(QTextStream &t);
     
  protected:
    void keyPressEvent(QKeyEvent *event);

  signals:
    void prevSnapshot();
    void nextSnapshot();

 public slots:
    void updateLevel(int n);
    void updateSearchString();
    void moduleChanged(int n);
};

#endif

