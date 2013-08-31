#include <QtGui>
#include "StateWindow.h"
#include <iostream>

using namespace std;

StateWindow::StateWindow() : QWidget() {
  QGridLayout *layout = new QGridLayout;

  // from frame info
  names[0] = "Frame #";
  names[1] = "Time";
  names[2] = "Memory Source";

  // from robot state
  names[3] = "WO SELF";
  names[4] = "RB Team";
  names[5] = "Robot ID";
  names[6] = "Role";

  // from game state
  names[7] = "State";
  names[8] = "GC Team";
  names[9] = "Penalty Seconds";
  names[10] = "Our Score";
  names[11] = "Their Score";
  names[12] = "Seconds Left";

  labels = new QLabel[NUM_ITEMS];
  values = new QLabel[NUM_ITEMS];

  // set items
  for (int i = 0; i < NUM_ITEMS; i++) {
    labels[i].setText(names[i]);
    values[i].setText(QString::number(0.0));

    // add to layout
    layout->addWidget(&labels[i], i, 0);
    layout->addWidget(&values[i], i, 1);
  }
  setLayout(layout);
  
  resize(120,200);

  setWindowTitle(tr("Robot/Game State"));
}

void StateWindow::update(Memory* memory) {

  // get frame info and fill in
  FrameInfoBlock* frameInfo = NULL;
  memory->getBlockByName(frameInfo, "vision_frame_info",false);

  // fill it in if we've got it
  if (frameInfo != NULL){
    values[0].setText(QString::number(frameInfo->frame_id,'f',0));
    values[1].setText(QString::number(frameInfo->seconds_since_start,'f',3));
    if (frameInfo->source == MEMORY_ROBOT)
      values[2].setText("Robot");
    else
      values[2].setText("Sim");
  } 
  // or not available
  else {
    values[0].setText("NA");
    values[1].setText("NA");
    values[2].setText("NA");
  } 

  // get robot state and fill in
  RobotStateBlock* robotState = NULL;
  memory->getBlockByName(robotState, "robot_state", false);

  // fill it in if we've got it
  if (robotState != NULL){
    values[3].setText(QString::number(robotState->WO_SELF,'f',0));
    if (robotState->team_ == TEAM_RED)
      values[4].setText("RED");
    else
      values[4].setText("BLUE");
    values[5].setText(QString::number(robotState->robot_id_,'f',0));
    values[6].setText(roleNames[robotState->role_].c_str());
  }
  // or not available
  else {
    values[3].setText("NA");
    values[4].setText("NA");
    values[5].setText("NA");
    values[6].setText("NA");
  } 

  // get game state and fill in
  GameStateBlock* gameState = NULL;
  memory->getBlockByName(gameState, "game_state", false);

  // fill it in if we've got it
  if (gameState != NULL){
    values[7].setText(stateNames[gameState->state].c_str());
    values[8].setText(QString::number(gameState->gameContTeamNum,'f',0));
    values[9].setText(QString::number(gameState->secsTillUnpenalised,'f',0));
    values[10].setText(QString::number(gameState->ourScore,'f',0));
    values[11].setText(QString::number(gameState->opponentScore,'f',0));
    values[12].setText(QString::number(gameState->secsRemaining,'f',0));
  }
  // or not available
  else {
    values[7].setText("NA");
    values[8].setText("NA");
    values[9].setText("NA");
    values[10].setText("NA");
    values[11].setText("NA");
    values[12].setText("NA");
  } 

 
}
