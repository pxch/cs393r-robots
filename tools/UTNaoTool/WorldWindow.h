#ifndef WORLD_WINDOW_H
#define WORLD_WINDOW_H

#include <QWidget>

#include "WorldGLWidget.h"

#include <memory/Memory.h>

class QCheckBox;
class QPushButton;
class QWidget;
class QMainWindow;

class WorldWindow : public QWidget {
 Q_OBJECT

  public:
    WorldWindow(QMainWindow* pa);
    ~WorldWindow();
      
    void updateMemory(Memory* mem);

    WorldGLWidget *world;

    QMainWindow* parent;

  public slots:
    void updateTitle(QString);

};

#endif
