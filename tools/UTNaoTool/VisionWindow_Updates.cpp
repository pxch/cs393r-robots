void VisionWindow::changeToRawTop() {
    emit cameraChanged(Camera::TOP);
    changeBigImage(RAW_IMAGE, IMAGE_TOP);
}
void VisionWindow::changeToRawBottom() {
    emit cameraChanged(Camera::BOTTOM);
    changeBigImage(RAW_IMAGE, IMAGE_BOTTOM);
}
void VisionWindow::changeToSegTop() {
    emit cameraChanged(Camera::TOP);
    changeBigImage(SEG_IMAGE, IMAGE_TOP);
}
void VisionWindow::changeToSegBottom() {
    emit cameraChanged(Camera::BOTTOM);
    changeBigImage(SEG_IMAGE, IMAGE_BOTTOM);
}
void VisionWindow::changeToHorizontalBlobTop() {
    emit cameraChanged(Camera::TOP);
    changeBigImage(HORIZONTAL_BLOB_IMAGE, IMAGE_TOP);
}
void VisionWindow::changeToHorizontalBlobBottom() {
    emit cameraChanged(Camera::BOTTOM);
    changeBigImage(HORIZONTAL_BLOB_IMAGE, IMAGE_BOTTOM);
}
void VisionWindow::changeToVerticalBlobTop() {
    emit cameraChanged(Camera::TOP);
    changeBigImage(VERTICAL_BLOB_IMAGE, IMAGE_TOP);
}
void VisionWindow::changeToVerticalBlobBottom() {
    emit cameraChanged(Camera::BOTTOM);
    changeBigImage(VERTICAL_BLOB_IMAGE, IMAGE_BOTTOM);
}
void VisionWindow::changeToObjTop() {
    emit cameraChanged(Camera::TOP);
    changeBigImage(OBJ_IMAGE, IMAGE_TOP);
}
void VisionWindow::changeToObjBottom() {
    emit cameraChanged(Camera::BOTTOM);
    changeBigImage(OBJ_IMAGE, IMAGE_BOTTOM);
}
void VisionWindow::changeToTransformedTop() {
    emit cameraChanged(Camera::TOP);
    changeBigImage(TRANSFORMED_IMAGE, IMAGE_TOP);
}
void VisionWindow::changeToTransformedBottom() {
    emit cameraChanged(Camera::BOTTOM);
    changeBigImage(TRANSFORMED_IMAGE, IMAGE_BOTTOM);
}


void VisionWindow::changeBigImage(int type, int cam) {
  currentBigImageType_ = type;
  currentBigImageCam_ = cam;
  _widgetAssignments[bigImage] = cam;
  bigImage->fill(Qt::black);
  redrawImages();
}

void VisionWindow::calibrationsUpdated() {
    update();
}

void VisionWindow::updateClassificationCheck(bool value) {
  doingClassification_ = value;
  if (doingClassification_ && getImageProcessor(bigImage)->getImg() == NULL){
    std::cout << "No classification without raw images!" << std::endl;
    doingClassification_ = false;
  }
  if (doingClassification_ && !streaming_)
    emit setCore(true);
  redrawImages();
}

void VisionWindow::updateCalibrationCheck(bool value) {
  doingCalibration_ = value;
  ImageProcessor *top = core_->vision_->top_processor_;
  ImageProcessor *bottom = core_->vision_->bottom_processor_;
  top->enableCalibration(value);
  bottom->enableCalibration(value);
  redrawImages();
}

void VisionWindow::updateTable(unsigned char *colorTable, int yIdx, int uIdx, int vIdx) {

  int ySen = classification->yDial->value();
  int uSen = classification->uDial->value();
  int vSen = classification->vDial->value();

  for (int y = max(yIdx - ySen, 0); y <= min(yIdx + ySen, 255); y++) {
    for (int u = max(uIdx - uSen, 0); u <= min(uIdx + uSen, 255); u++) {
      for (int v = max(vIdx - vSen, 0); v <= min(vIdx + vSen, 255); v++) {
          ColorTableMethods::assignColor(colorTable, y,u,v, (Color)classification->colorCombo->currentIndex());
      }
    }
  }

}

void VisionWindow::updateClicked(int xIdx, int yIdx, int buttonIdx){
  if(!initialized_) return;
  int image = currentBigImageCam_;
  ImageProcessor* processor = getImageProcessor(image);
  unsigned char* colorTable = processor->getColorTable();
  const ImageParams& iparams = processor->getImageParams();

  if (doingCalibration_) {
    Sample s; s.x = xIdx; s.y = yIdx;
    if(image == IMAGE_TOP)
      s.camera = Camera::TOP;
    else
      s.camera = Camera::BOTTOM;
    emit calibrationSampleAdded(s);
    redrawImages();
  }

  if (doingClassification_) {
    if (buttonIdx == Qt::LeftButton) {

      //for(int i=0; i < LUT_SIZE; i++)
        //std::cout << colorTable[i] << "\,";
      //std::cout << "DONE\n";
      memcpy(tempTable,colorTable,LUT_SIZE);
      ColorTableMethods::xy2yuv(processor->getImg(), xIdx, yIdx, iparams.width, currentY_, currentU_, currentV_);
      updateTable(colorTable, currentY_, currentU_, currentV_);
      //for(int i=0; i < LUT_SIZE; i++)
        //std::cout << tempTable[i] << "\,";
        //sstd::cout << "\n";
      colorUpdateAvailable_ = true;
      redrawImages();
      processor->processFrame();
      memcpy(colorTable,tempTable,LUT_SIZE);

    } else if (buttonIdx == Qt::RightButton && colorUpdateAvailable_) {

      memcpy(undoTable, colorTable, LUT_SIZE);
      undoImage_ = image;
      updateTable(colorTable, currentY_, currentU_, currentV_);
      colorUpdateAvailable_ = false;

      redrawImages();
    }
  }
}


void VisionWindow::updateToolTip(int image) {
  ImageProcessor* processor = getImageProcessor(image);
  if (!toolCheck->isChecked() || !core_ || !core_->vision_ || !((UTMainWnd*)parent_)->runCoreRadio->isChecked()) {
    QToolTip::hideText();
    return;
  }

  QString text="";

  // raw image or seg image: tool tip is pixel info
  if (currentBigImageType_ == RAW_IMAGE || currentBigImageType_ == SEG_IMAGE) {

    if (currentBigImageType_ == RAW_IMAGE && overlayCheck->isChecked() && mouseOverLineIndex_ != -1) {
      FieldLine *line = NULL;
      if (mouseOverLineType_ == 0)
        line = processor->line_detector_->fieldLines[mouseOverLineIndex_];
      else if (mouseOverLineType_ == 1)
        line = processor->goal_detector_->yellowPosts[mouseOverLineIndex_];

      text+=xyLabel->text()+"\n";
      text+="Line: "+QString::number(mouseOverLineIndex_)+" Type: "+ QString::number(mouseOverLineType_)+ "\n";
      text+="Pts: "+ QString::number(line->Points)+"\n";
      text+="Length, Width: "+ QString::number(line->length) + ", " + QString::number(line->width)+"\n";
      text+="Offset "+QString::number(line->Offset)+"\n";
      text+="Slope, Angle "+QString::number(line->Slope)+", "+QString::number(RAD_T_DEG*line->Angle)+"\n";
      text+="RateSlope " +QString::number(line->rateSlope)+"\n";
      text+="Circle: "+QString::number(line->isCircle);
      text+=" Curve: "+QString::number(line->isCurve) +" OnCircle: "+QString::number(line->onCircle)+"\n";
      text+="ValidLine: "+QString::number(line->ValidLine);

    } else {
      text = xyLabel->text()+"\n"+rgbLabel->text()+"\n"+yuvLabel->text()+"\n"+segLabel->text();
    }
  }
  // blob image: tool tip is blobs, lines
  else if (currentBigImageType_ == VERTICAL_BLOB_IMAGE) {
    // show blob info
    if (mouseOverBlobIndex_ != -1 && mouseOverBlobType_ == c_ORANGE) {
      Blob *blob = &(processor->blob_detector_->horizontalBlob[c_ORANGE][mouseOverBlobIndex_]);
      text+=xyLabel->text() + "\n";
      text+="Blob: "+QString::number(mouseOverBlobIndex_) + "\n";
      text+="A,H,W: " + QString::number(blob->dx * blob->dy) + "," + QString::number(blob->dy) + "," + QString::number(blob->dx) + "\n";
      text += "Pix R: " + QString::number((double)blob->correctPixelRatio);
    } else if (mouseOverBlobIndex_ != -1 && mouseOverBlobType_ == c_WHITE) {
      Blob *blob = &(processor->blob_detector_->horizontalBlob[c_WHITE][mouseOverBlobIndex_]);
      text+=xyLabel->text() + "\n";
      text+="Blob: "+QString::number(mouseOverBlobIndex_) + "\n";
      text+="Starts: ("+QString::number(blob->xi) + "," + QString::number(blob->yi) +")"+"\n";
      text+="Ends: ("+QString::number(blob->xf) + "," + QString::number(blob->yf) +")"+"\n";
      text+="Diff: Start: "+QString::number(blob->diffStart)+" End: "+QString::number(blob->diffEnd) + "\n";
      text+="Width: Start: "+QString::number(blob->widthStart)+" End: "+QString::number(blob->widthEnd) + "\n";
      text+="Double Diff: "+QString::number(blob->doubleDiff)+"\n";
      text+="LPs: "+QString::number(blob->lpCount);
    }
  } else if (currentBigImageType_ == HORIZONTAL_BLOB_IMAGE) {
    if (mouseOverBlobIndex_ != -1 && mouseOverBlobType_ == c_WHITE) {
      Blob *blob = &(processor->blob_detector_->verticalBlob[c_WHITE][mouseOverBlobIndex_]);
      text+=xyLabel->text() + "\n";
      text+="Blob: "+QString::number(mouseOverBlobIndex_) + "\n";
      text+="Starts: ("+QString::number(blob->xi) + "," + QString::number(blob->yi) +")"+"\n";
      text+="Ends: ("+QString::number(blob->xf) + "," + QString::number(blob->yf) +")"+"\n";
      text+="Diff: Start: "+QString::number(blob->diffStart)+" End: "+QString::number(blob->diffEnd) + "\n";
      text+="Width: Start: "+QString::number(blob->widthStart)+" End: "+QString::number(blob->widthEnd) + "\n";
      text+="Double Diff: "+QString::number(blob->doubleDiff)+"\n";
      text+="LPs: "+QString::number(blob->lpCount);
    }
  }

  // obj image: show world object info
  else if (currentBigImageType_ == OBJ_IMAGE) {
    if (mouseOverObjectIndex_==-1) return;
    WorldObject* wo=&world_object_block_->objects_[mouseOverObjectIndex_];

    text+=xyLabel->text()+"\n";
    text+="Obj: "+QString::number(mouseOverObjectIndex_)+"   Line: "+QString::number(wo->fieldLineIndex)+ "\n";
    text+="Dis: "+ QString::number(wo->visionDistance)+ "\n";
    text+="Bea: "+ QString::number(RAD_T_DEG*wo->visionBearing)+ "\n";
    text+="Ele: "+ QString::number(RAD_T_DEG*wo->visionElevation) +"\n";

    text+="Rel X: "+ QString::number(cos(wo->bearing)*wo->distance)+ "\n";
    text+="Rel Y: "+ QString::number(sin(wo->bearing)*wo->distance);

    FieldLine* line;
    if (wo->fieldLineIndex != -1){
      if (wo->isGoal())
        line = processor->goal_detector_->yellowPosts[wo->fieldLineIndex];
      else
        line = processor->line_detector_->fieldLines[wo->fieldLineIndex];

      text+="Line: "+QString::number(wo->fieldLineIndex)+"\n";
      text+="Pts, Width: "+ QString::number(line->Points) + ", " + QString::number(line->width)+"\n";
      text+="Offset "+QString::number(line->Offset)+"\n";
      text+="Slope, Angle "+QString::number(line->Slope)+", "+QString::number(RAD_T_DEG*line->Angle)+"\n";
      text+="RateSlope " +QString::number(line->rateSlope)+"\n";
      text+="Circle "+QString::number(line->isCircle)+"\n";
      text+="ValidLine: "+QString::number(line->ValidLine);
    }
  }

  if (text!="") {
    QPoint p = QCursor::pos();
    QToolTip::showText(p, text);
  } else {
    QToolTip::hideText();
  }
}

