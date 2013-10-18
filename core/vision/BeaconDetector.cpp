#include "BeaconDetector.h"

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier,
		BlobDetector*& blob_detector) :
		DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(
				blob_detector) {
}

void BeaconDetector::detectBeacon() {
	WorldObject* beacon_p_y = &vblocks_.world_object->objects_[WO_BEACON_PINK_YELLOW];
	WorldObject* beacon_y_p = &vblocks_.world_object->objects_[WO_BEACON_YELLOW_PINK];
	WorldObject* beacon_b_y = &vblocks_.world_object->objects_[WO_BEACON_BLUE_YELLOW];
	WorldObject* beacon_y_b = &vblocks_.world_object->objects_[WO_BEACON_YELLOW_BLUE];
	WorldObject* beacon_p_b = &vblocks_.world_object->objects_[WO_BEACON_PINK_BLUE];
	WorldObject* beacon_b_p = &vblocks_.world_object->objects_[WO_BEACON_BLUE_PINK];

	for(int i = 0; i < blob_detector_->horizontalBlob[c_PINK].size(), ++i) {

	}
}
