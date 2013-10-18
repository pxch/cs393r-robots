#ifndef BEACONDETECTOR_H
#define BEACONDETECTOR_H

#include <string>
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

	void detectBeacon();

	bool isVerticalConnected(Blob& blob1, Blob& blob2);

	void formBeacon(WorldObject* beacon, Blob& blob1, Blob& blob2);

private:
	TextLogger* textlogger;
	Classifier* classifier_;
	BlobDetector* blob_detector_;

	void detectBeacon(WorldObject* beacon, int color1, int color2,
			std::string color1_str, std::string color2_str);

};

#endif
