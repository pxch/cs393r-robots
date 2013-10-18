#ifndef BEACONDETECTOR_H
#define BEACONDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/Classifier.h>
#include <vision/structures/BallCandidate.h>

class BeaconDetector: public ObjectDetector {
public:
	BeaconDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier,
			BlobDetector*& blob_detector);
	void init(TextLogger* tl) {
		textlogger = tl;
	}
	;
	void detectBeacon();

private:
	TextLogger* textlogger;
	Classifier* classifier_;
	BlobDetector* blob_detector_;
};

#endif
