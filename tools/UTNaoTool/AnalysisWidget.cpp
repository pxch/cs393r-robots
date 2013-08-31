#include "AnalysisWidget.h"

AnalysisWidget::AnalysisWidget(QWidget* parent) : QWidget(parent), log_(0) {
    setupUi(this);

    colorStrings[c_UNDEFINED]="Undefined";
    colorStrings[c_FIELD_GREEN]="Field Green";
    colorStrings[c_WHITE]="White";
    colorStrings[c_ORANGE]="Orange";
    colorStrings[c_PINK]="Pink";
    colorStrings[c_BLUE]="Blue";
    colorStrings[c_YELLOW]="Yellow";
    colorStrings[c_ROBOT_WHITE] = "Robot White";

    for (int i=1; i<NUM_COLORS; i++) {
        colorBox->addItem(colorStrings[i]);
    }

    connect(analyzeButton, SIGNAL(clicked()), this, SLOT(analyze()));
    connect(pruneButton, SIGNAL(clicked()), this, SLOT(prune()));
    connect(undoButton, SIGNAL(clicked()), this, SLOT(undo()));
    connect (colorBox, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(colorBoxIndexChanged(const QString &)));
    txtFalseCandidates->setVisible(false);
    txtFalseBalls->setVisible(false);
    txtMissingBalls->setVisible(false);
    lblFalseCandidates->setVisible(false);
    lblFalseBalls->setVisible(false);
    lblMissingBalls->setVisible(false);
    txtBallStats->setVisible(false);
}

void AnalysisWidget::analyze(){
    analyzer_.setAnnotations(annotations_);
    ImageProcessor* processor = (currentCamera_ == Camera::TOP ? topProcessor_ : bottomProcessor_);
    unsigned char* colorTable = processor->getColorTable();
    analyzer_.setColorTable(colorTable);
    fpcText->setText(QString::number(analyzer_.falsePositiveCount(selectedColor_)));
    fprText->setText(QString::number(analyzer_.falsePositiveRate(selectedColor_)));
    fncText->setText(QString::number(analyzer_.falseNegativeCount(selectedColor_)));
    fnrText->setText(QString::number(analyzer_.falseNegativeRate(selectedColor_)));
    if(selectedColor_ == c_ORANGE) {
      ballstats stats = getBallStatistics();
      txtFalseBalls->setText(QString::number(stats.falseBalls));
      txtMissingBalls->setText(QString::number(stats.missingBalls));
      txtFalseCandidates->setText(QString::number(stats.falseCandidates));
    }
    pointCountText->setText(QString::number(analyzer_.colorTablePointCount(selectedColor_)));
}

void AnalysisWidget::handleNewLogLoaded(Log* log){
  std::vector<ImageParams> iparams = (currentCamera_ == Camera::TOP ? log->getTopParams() : log->getBottomParams());
    std::vector<unsigned char*> images = (currentCamera_ == Camera::TOP ? log->getRawTopImages() : log->getRawBottomImages());
    analyzer_.setImages(images, iparams);
    log_ = log;
}

void AnalysisWidget::setAnnotations(std::vector<Annotation*> annotations){
    annotations_ = annotations;
}

void AnalysisWidget::setImageProcessors(ImageProcessor* top, ImageProcessor* bottom){
    topProcessor_ = top;
    bottomProcessor_ = bottom;
}

void AnalysisWidget::setCurrentCamera(Camera::Type camera){
    currentCamera_ = camera;
    if(log_) {
      std::vector<ImageParams> iparams = (currentCamera_ == Camera::TOP ? log_->getTopParams() : log_->getBottomParams());
      std::vector<unsigned char*> images = (currentCamera_ == Camera::TOP ? log_->getRawTopImages() : log_->getRawBottomImages());
        analyzer_.setImages(images, iparams);
    }
}

void AnalysisWidget::colorBoxIndexChanged(const QString& text) {
    (void)text; // kill warning
    int index = colorBox->currentIndex();
    selectedColor_ = (Color)(index + 1);
    if(selectedColor_ == c_ORANGE) {
        txtFalseBalls->setVisible(true);
        txtFalseCandidates->setVisible(true);
        txtMissingBalls->setVisible(true);
        lblFalseBalls->setVisible(true);
        lblFalseCandidates->setVisible(true);
        lblMissingBalls->setVisible(true);
        txtBallStats->setVisible(true);
    }
    else {
        txtFalseBalls->setVisible(false);
        txtFalseCandidates->setVisible(false);
        txtMissingBalls->setVisible(false);
        lblFalseBalls->setVisible(false);
        lblFalseCandidates->setVisible(false);
        lblMissingBalls->setVisible(false);
        txtBallStats->setVisible(false);
    }

}

void AnalysisWidget::prune() {
    analyzer_.setAnnotations(annotations_);
    ImageProcessor* processor = (currentCamera_ == Camera::TOP ? topProcessor_ : bottomProcessor_);
    unsigned char* colorTable = processor->getColorTable();
    analyzer_.setColorTable(colorTable);
    float amount = (float)prunePercentBox->value() / 100;
    analyzer_.removeCriticalPoints(selectedColor_, amount);
    emit colorTableGenerated();
    analyze();
    undoButton->setEnabled(true);
    std::cout << "Pruned bad assignments for " << colorStrings[selectedColor_].toStdString() << "\n";
}

void AnalysisWidget::undo() {
    analyzer_.undo();
    emit colorTableGenerated();
    analyze();
    if(!analyzer_.hasUndo())
        undoButton->setEnabled(false);
}

void AnalysisWidget::handleColorTableGenerated(){
    analyzer_.clear();
    undoButton->setEnabled(false);
}

ballstats AnalysisWidget::getBallStatistics() {
    ballstats stats;
    QString statsText;
    for(uint16_t frame = 0; frame < log_->size(); frame++){
        ImageProcessor* processor = (currentCamera_ == Camera::TOP ? topProcessor_ : bottomProcessor_);
        Memory& memory = (*log_)[frame];
        core_->updateMemory(&memory);
        core_->localization_->processFrame();
        core_->vision_->processFrame();
        std::vector<BallCandidate*> candidates = processor->getBallCandidates();
        BallCandidate* best = processor->getBestBallCandidate();
        for(uint16_t j = 0; j < candidates.size(); j++) {
            BallCandidate* candidate = candidates[j];
            bool valid = false;
            for(uint16_t k = 0; k < annotations_.size(); k++) {
                Annotation* a = annotations_[k];
                if(!a->isInFrame(frame)) continue;
                if(a->getColor() != c_ORANGE) continue;
                if(a->enclosesPoint(frame, candidate->centerX, candidate->centerY))
                    valid = true;
            }
            if(candidate == best && !valid) {
              stats.falseBalls++;
              statsText += "Frame " + QString::number(frame) + ", False Ball: # " + QString::number(j) + "\n";
            }
            else if (!valid) {
              stats.falseCandidates++;
              statsText += "Frame " + QString::number(frame) + ", False Candidate: # " + QString::number(j) + "\n";
            }
        }
        bool bestFound = false;
        bool ballAnnotated = false;
        for(uint16_t k = 0; k < annotations_.size(); k++) {
            Annotation* a = annotations_[k];
            if(!a->isInFrame(frame)) continue;
            if(a->getColor() != c_ORANGE) continue;
            if(a->getCamera() != currentCamera_) continue;
            ballAnnotated = true;
            bool found = false;
            for(uint16_t j = 0; j < candidates.size(); j++) {
                BallCandidate* candidate = candidates[j];
                if(candidate == best) continue;
                if(a->enclosesPoint(frame, candidate->centerX, candidate->centerY))
                    found = true;
            }
            if(!found)
                stats.missingCandidates++;
            if(best && a->enclosesPoint(frame, best->centerX, best->centerY))
                bestFound = true;
        }
        if(ballAnnotated && !bestFound) {
            stats.missingBalls++;
            statsText += "Frame " + QString::number(frame) + ", Missing Ball\n";
        }
    }
    txtBallStats->setText(statsText);
    emit memoryChanged();
    return stats;
}

void AnalysisWidget::setCore(VisionCore* core){
    core_ = core;
}

void AnalysisWidget::handleNewLogFrame(int frame) {
    currentFrame_ = frame;
}
