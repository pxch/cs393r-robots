#include <common/RobotInfo.h>

ImageParams::ImageParams(Camera::Type camera) {
  if(camera == Camera::TOP) {
    width = 1280;
    height = 960;
    defaultHorizontalStepScale = 3;
    defaultVerticalStepScale = 2;
  }
  else {
    width = 320;
    height = 240;
    defaultHorizontalStepScale = 0;
    defaultVerticalStepScale = 0;
  }

  // Original Parameters
  //width = 640;
  //height = 480;
  //defaultHorizontalStepScale = 2;
  //defaultVerticalStepScale = 1;

  size = width * height;
  rawSize = size * 2;
  factor = width / 160;
  origFactor = width / 640.0f;
}
