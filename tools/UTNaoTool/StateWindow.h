#ifndef STATE_WINDOW_H
#define STATE_WINDOW_H

#include <QWidget>

#include <memory/Memory.h>
#include <memory/RobotStateBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/FrameInfoBlock.h>

#include <common/States.h>
#include <common/Roles.h>

class QLabel;
class QWidget;

const int NUM_ITEMS = 13;

class StateWindow : public QWidget {
 Q_OBJECT

  public:
  StateWindow();
    
  void update(Memory* memory);
  QLabel* labels;
  QLabel* values;
  QString names[NUM_ITEMS];

};

#endif
