#include <iostream>

#include <vision/GoalDetector.h>

GoalDetector::GoalDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier,
		BlobDetector*& blob_detector, LineDetector*& line_detector) :
		DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(
				blob_detector), line_detector_(line_detector) {
}

void GoalDetector::detectGoal() {
	// XXX: decide whose goal this is
	WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

	int goalX, goalY, goalLX, goalLY, goalRX, goalRY, goalDirection;
	float visionRatio;
	bool seen;

	if (camera_ == Camera::BOTTOM) {
		goal->seen = false;
		return;

	} else {

		findGoal(visionRatio, goalX, goalY, goalLX, goalLY, goalRX, goalRY,
				goalDirection, seen);

		if (seen) {

			goal->seen = true;

			goal->imageCenterX = goalX;
			goal->imageCenterY = goalY;

			goal->lCenterX = goalLX;
			goal->lCenterY = goalLY;

			goal->rCenterX = goalRX;
			goal->rCenterY = goalRY;

			goal->goalCenterDirection = goalDirection;

			goal->radius = visionRatio;

			goal->fromTopCamera = true;

		}

	}
}

void GoalDetector::findGoal(float &visionRatio, int &goalX, int &goalY,
		int &goalLX, int &goalLY, int &goalRX, int &goalRY, int &goalDirection,
		bool &seen) {

	//goalDirection == 1 --> goal on the left
	//goalDirection == 2 --> goal on the right

	goalX = goalY = 0;
	visionRatio = 0;
	goalLX = goalLY = goalRX = goalRY = 0;
	goalDirection = 0;

	int total = 0;
	for (int x = 0; x < iparams_.width; x++) {
		for (int y = 0; y < iparams_.height; y++) {
			if (getSegPixelValueAt(x,y) == c_BLUE) {
				goalX += x, goalY += y, total++;
			}
		}
	}
	if (total > 2500) {
		seen = true;
		goalX /= total, goalY /= total;
		visionRatio = float(total) / float(iparams_.width)
		/ float(iparams_.height);

		int lTotal = 0;
		int rTotal = 0;

		for ( int x = 0; x < goalX; x ++) {
			for ( int y = 0; y < iparams_.height; y ++) {
				if (getSegPixelValueAt(x,y) == c_BLUE) {
					goalLX += x, goalLY += y, lTotal ++;
				}
			}
		}
		goalLX /= lTotal, goalLY /= lTotal;

		for ( int x = goalX; x < iparams_.width; x ++) {
			for ( int y = 0; y < iparams_.height; y ++) {
				if (getSegPixelValueAt(x,y) == c_BLUE) {
					goalRX += x, goalRY += y, rTotal ++;
				}
			}
		}
		goalRX /= rTotal, goalRY  /= rTotal;

		if (lTotal > rTotal) {
			goalDirection = 1;
		}
		else {
			goalDirection = 2;
		}
	} else {
		seen = false;
	}
}
