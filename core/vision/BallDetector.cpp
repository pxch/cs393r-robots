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

	classifier_->classifyImage(color_table_);
	classifier_->constructRuns();
	blob_detector_->formBlobs(c_ORANGE);
	BlobCollection blobs = blob_detector_->horizontalBlob[c_ORANGE];
	if (blobs.size() > 0) {
		printf("found %i blobs\n", blobs.size());
		for (int i = 0; i < blobs.size(); i++) {
			Blob& b = blobs[i];
			int blobArea = abs(b.xf - b.xi) * abs(b.yf - b.yi);
			if (blobArea >= 15) {
				printf("box area: %d\n", blobArea);
				printf("IMAGE_POS %i %i\n", b.avgX, b.avgY);

				Position p = cmatrix_.getWorldPosition(b.avgX, b.avgY);
				printf("WORLD_POS %f %f %f\n", p.x, p.y, p.z);
			}
		}
	}

}

