#ifndef BALLDETECTOR_H
#define BALLDETECTOR_H

#include <memory/TextLogger.h>
#include <vision/BlobDetector.h>
#include <vision/ObjectDetector.h>
#include <vision/Classifier.h>
#include <vision/structures/BallCandidate.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class BallDetector: public ObjectDetector {
public:
	BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier,
			BlobDetector*& blob_detector);

	void init(TextLogger* tl) {
		textlogger = tl;
	}

	void detectBall();

	BallCandidate candidates[MAX_BALL_CANDS];

	int candidateCount;

private:
	TextLogger* textlogger;

	Classifier* classifier_;

	BlobDetector* blob_detector_;

	void findBall(int& imageX, int& imageY, bool& seen);
	void findBallMaxOrange(int& imageX, int& imageY, bool& seen);
};

#endif
