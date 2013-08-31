#ifndef BLOBDETECTOR_H
#define BLOBDETECTOR_H


#include <memory/TextLogger.h>
#include <vision/Classifier.h>
#include <vision/structures/Blob.h>
#include <vision/ObjectDetector.h>
#include <vision/structures/Blob.h>

typedef std::vector<Blob> BlobCollection;
  
class BlobDetector : public ObjectDetector {
 public:
  BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier);
  void init(TextLogger* tl){textlogger = tl;};
  std::vector<BlobCollection> horizontalBlob, verticalBlob;
 private:
  TextLogger* textlogger;
  Classifier* classifier_;
};

#endif
