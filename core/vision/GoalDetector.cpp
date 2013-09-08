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

	findGoal(visionRatio, goalX, goalY);

	goal->imageCenterX = goalX;
	goal->imageCenterY = goalY;

	goal->radius = visionRatio;

	goal->seen = true;
}

void GoalDetector::findGoal(float &visionRatio, int &goalX, int &goalY) {
	goalX = goalY = 0;
	int total = 0;
	for (int x = 0; x < iparams_.width; x++)
		for (int y = 0; y < iparams_.height; y++)
			if (getSegPixelValueAt(x,y) == c_ORANGE)
				goalX += x, goalY += y, total++;
	goalX /= total, goalY /= total;
	visionRatio = total / iparams_.width / iparams_.height;
}
