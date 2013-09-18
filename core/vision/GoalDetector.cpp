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

	int goalX, goalY;
	float visionRatio;
	bool seen;

	if (camera_ == Camera::BOTTOM) {

		goal->seen = false;

	} else {

		findGoal(visionRatio, goalX, goalY, seen);

		if (seen) {

			goal->seen = true;

			goal->imageCenterX = goalX;
			goal->imageCenterY = goalY;

			goal->radius = visionRatio;

			goal->fromTopCamera = true;

		}

	}
}

void GoalDetector::findGoal(float &visionRatio, int &goalX, int &goalY,
		bool &seen) {
	goalX = goalY = 0;
	visionRatio = 0;
	int total = 0;
	for (int x = 0; x < iparams_.width; x++) {
		for (int y = 0; y < iparams_.height; y++) {
			if (getSegPixelValueAt(x,y) == c_BLUE) {
				goalX += x, goalY += y, total++;
			}
		}
	}
	if (total) {
		goalX /= total, goalY /= total;
		visionRatio = float(total) / float(iparams_.width)
				/ float(iparams_.height);
	} else {
		seen = false;
	}
}
