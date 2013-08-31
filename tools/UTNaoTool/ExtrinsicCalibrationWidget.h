#ifndef EXTRINSIC_CALIBRATION_WIDGET_H
#define EXTRINSIC_CALIBRATION_WIDGET_H

#include <iostream>
#include "ui_ExtrinsicCalibrationWidget.h"
#include <common/RobotInfo.h>
#include <vector>
#include <vision/structures/Sample.h>
#include <vision/RobotCalibration.h>
#include <calibration/ExtrinsicCalibrator.h>
#include <memory/WorldObjectBlock.h>
#include <common/RobotInfo.h>
#include <common/RobotDimensions.h>

class ExtrinsicCalibrationWidget : public QWidget, public Ui_UTExtrinsicCalibrationWidget {
  Q_OBJECT
  private:
    WorldObjectBlock* world_object_block_;
    vector<Sample> samples_;
    void initializeItems();
  public:
    ExtrinsicCalibrationWidget(QWidget* parent);
    std::string calibration_file_;
    void saveCalibration(std::string);
    void loadCalibration(std::string);
    void loadCalibration(const RobotCalibration& cal, bool includePose = true);
    RobotCalibration getCalibration() const;
    void setWorldObjectBlock(WorldObjectBlock* block);
    std::vector<Sample> getSamples() const;
  public slots:
    void resetParameters();
    void clear();
    void save();
    void saveAs();
    void load();
    void optimizeCalibration();
    void addSample(Sample);
  protected slots:
    void toggleList(int state, QWidget* box);
signals:
    void calibrationsUpdated();
};

#endif /* end of include guard: EXTRINSIC_CALIBRATION_WIDGET_H */
