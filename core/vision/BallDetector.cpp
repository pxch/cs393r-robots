#include <iostream>

#include "BallDetector.h"

using namespace Eigen;

#define getball() (&vblocks_.world_object->objects_[WO_BALL])
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])
#define getframe() vblocks_.frame_info->frame_id

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier,
		BlobDetector*& blob_detector) :
		DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(
				blob_detector) {
	candidateCount = 0;
}

void BallDetector::detectBall() {
	WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

	int imageX, imageY;
	bool seen;

	if (camera_ == Camera::BOTTOM) {

		ball->seen = false;

		findBall(imageX, imageY, seen);
		ball->seen = seen;
		ball->imageCenterX = imageX;
		ball->imageCenterY = imageY;

		if (ball->seen) {
			ball->fromTopCamera = false;
		}

	} else {

		if (ball->seen == false) {

			findBall(imageX, imageY, seen);
			ball->seen = seen;
			ball->imageCenterX = imageX;
			ball->imageCenterY = imageY;

			if (ball->seen) {
				ball->fromTopCamera = true;
			}

		}
	}

}

void BallDetector::findBall(int& imageX, int& imageY, bool& seen) {
	imageX = imageY = 0;
	int total = 0;
	for (int x = 0; x < iparams_.width; x++)
		for (int y = 0; y < iparams_.height; y++)
			if (getSegPixelValueAt(x, y) == c_ORANGE)
				imageX += x, imageY += y, total++;
	if (total > 0) {
		imageX /= total, imageY /= total;
		seen = true;
	} else
		seen = false;
}

