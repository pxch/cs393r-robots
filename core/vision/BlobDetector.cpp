#include "BlobDetector.h"

BlobDetector::BlobDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier) :
		DETECTOR_INITIALIZE, classifier_(classifier) {
	for (int i = 0; i < NUM_COLORS; i++)
		horizontalBlob.push_back(std::vector<Blob>());

	for (int i = 0; i < NUM_COLORS; i++)
		verticalBlob.push_back(std::vector<Blob>());
}



