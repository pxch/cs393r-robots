#include "Classifier.h"
#include <iostream>

Classifier::Classifier(const VisionBlocks& vblocks, const VisionParams& vparams, const ImageParams& iparams, const Camera::Type& camera) :
    vblocks_(vblocks), vparams_(vparams), iparams_(iparams), camera_(camera), initialized_(false) {
  segImg_ = new unsigned char[iparams.size];
  segImgLocal_ = segImg_;

  horizontalPoint = new VisionPoint**[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    horizontalPoint[i] = new VisionPoint*[iparams_.height];
    for(int j = 0; j < iparams_.height; j++)
      horizontalPoint[i][j] = new VisionPoint[iparams_.width];
  }
  verticalPoint = new VisionPoint**[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    verticalPoint[i] = new VisionPoint*[iparams_.width];
    for(int j = 0; j < iparams_.width; j++)
      verticalPoint[i][j] = new VisionPoint[iparams_.height];
  }
  horizontalPointCount = new uint32_t*[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++) {
    horizontalPointCount[i] = new uint32_t[iparams_.height];
  }
  verticalPointCount = new uint32_t*[NUM_COLORS];
  for(int i=0;i<NUM_COLORS;i++)
    verticalPointCount[i] = new uint32_t[iparams_.width];

  clearPoints();
  setStepScale(iparams_.defaultHorizontalStepScale, iparams_.defaultVerticalStepScale);
}

Classifier::~Classifier() {
  for(int i=0;i<NUM_COLORS;i++) {
    for(int j = 0; j < iparams_.height; j++)
      delete [] horizontalPoint[i][j];
    delete [] horizontalPoint[i];
  }
  delete [] horizontalPoint;

  for(int i=0;i<NUM_COLORS;i++) {
    for(int j = 0; j < iparams_.width; j++)
      delete [] verticalPoint[i][j];
    delete [] verticalPoint[i];
  }
  delete [] verticalPoint;
  
  for(int i=0;i<NUM_COLORS;i++)
    delete [] horizontalPointCount[i];
  delete [] horizontalPointCount;
  
  for(int i=0;i<NUM_COLORS;i++)
    delete [] verticalPointCount[i];
  delete [] verticalPointCount;
  delete [] segImgLocal_;
}

bool Classifier::setImagePointers() {
  bool imageLoaded = vblocks_.image->loaded_;
  if(vblocks_.image == NULL) {
    printf("No image block loaded! Classification failed.\n");
    return false;
  }
  if(vblocks_.robot_vision == NULL) {
    printf("No vision block loaded! Classification failed.\n");
    return false;
  }
  if(camera_ == Camera::TOP) {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgTop(segImg_);
      img_ = vblocks_.image->getImgTop();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->loaded_) {
      segImg_ = vblocks_.robot_vision->getSegImgTop();
    }
    #endif
  }
  else {
    #ifdef TOOL
    if(imageLoaded) {
    #endif
      vblocks_.robot_vision->setSegImgBottom(segImg_);
      img_ = vblocks_.image->getImgBottom();
      if(!img_) return false;
    #ifdef TOOL
    } else if(vblocks_.robot_vision->loaded_) {
      segImg_ = vblocks_.robot_vision->getSegImgBottom();
    }
    #endif
  }
  if(!initialized_) {
    #ifdef TOOL
    if(imageLoaded)
    #endif
    memset(segImg_, c_UNDEFINED, sizeof(unsigned char) * iparams_.size);
    initialized_ = true;
  }
  return true;
}

bool Classifier::classifyImage(unsigned char *colorTable) {
  if(!setImagePointers()) return false;
  FocusArea area(0, 0, iparams_.width - 1, iparams_.height - 1);
  classifyImage(area, colorTable);
  return true;
}

void Classifier::classifyImage(const std::vector<FocusArea>& areas, unsigned char *colorTable) {
  if(!setImagePointers()) return;
  for(unsigned int i = 0; i < areas.size(); i++)
    classifyImage(areas[i], colorTable);
}


void Classifier::classifyImage(const FocusArea& area, unsigned char* colorTable){
  bool imageLoaded = vblocks_.image->loaded_;
  if(!imageLoaded) {
    visionLog((20, "Classifying with no raw image"));
  }
  colorTable_ = colorTable;
  for (int y = area.y1; y <= area.y2; y += vstep_) {
    for(int x = area.x1; x <= area.x2; x += hstep_) {
      Color c;
#ifdef TOOL
      if (imageLoaded) // if a raw image is available
#endif
      {
        c = ColorTableMethods::xy2color(img_, colorTable, x, y, iparams_.width);
        segImg_[iparams_.width * y + x] = c;
      }
    }
  }
}

void Classifier::clearPoints(int colorFlags) {
  // Reset Vertical Point Counts
  for (int z = 0; z < NUM_COLORS; z++) {
    if(!isInFlags(z, colorFlags)) continue;
    memset(verticalPointCount[z], 0, sizeof(uint32_t) * iparams_.width);
  }

  // Reset Horizontal Point Counts
  for (int z = 0; z < NUM_COLORS; z++) {
    if(!isInFlags(z, colorFlags)) continue;
    memset(horizontalPointCount[z], 0, sizeof(uint32_t) * iparams_.height);
  }
}

void Classifier::setStepScale(int h, int v){
    hstep_ = (1 << h);
    vstep_ = (1 << v);
    hscale_ = h;
    vscale_ = v;
}

void Classifier::getStepSize(int& h, int& v){
    h = hstep_;
    v = vstep_;
}

void Classifier::getStepScale(int& h, int& v){
    h = hscale_;
    v = vscale_;
}

