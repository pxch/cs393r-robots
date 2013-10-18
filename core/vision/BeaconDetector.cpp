#include "BeaconDetector.h"
#include <iostream>

using namespace Eigen;

BeaconDetector::BeaconDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier,
		BlobDetector*& blob_detector) :
		DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(
				blob_detector) {
}

bool BeaconDetector::isVerticalConnected(Blob& blob1, Blob& blob2) {
	int vertical_dist_thres = 0;
	if (camera_ == Camera::TOP)
		vertical_dist_thres = 20;
	else
		vertical_dist_thres = 10;
	float horizontal_inter_thres = 0.8;

	int vertical_dist = 0;
	float horizontal_inter = 0.0;

	vertical_dist = abs(blob1.yf - blob2.yi);
	if (abs(blob1.xi - blob2.xf) > abs(blob1.xf - blob2.xi)) {
		horizontal_inter = float(abs(blob1.xf - blob2.xi))
				/ float(abs(blob1.xi - blob2.xf));
	} else {
		horizontal_inter = float(abs(blob1.xi - blob2.xf))
				/ float(abs(blob1.xf - blob2.xi));
	}
	if (vertical_dist <= vertical_dist_thres
			&& horizontal_inter >= horizontal_inter_thres)
		return true;
	else
		return false;
}

void BeaconDetector::formBeacon(WorldObject* beacon, Blob& blob1, Blob& blob2) {
	beacon->seen = true;
	if (abs(blob1.xi - blob2.xf) > abs(blob1.xf - blob2.xi))
		beacon->width = abs(blob1.xi - blob2.xf);
	else
		beacon->width = abs(blob1.xf - blob2.xi);
	beacon->height = blob2.yf - blob1.yi;
	beacon->imageCenterX = float(
			blob1.correctPixelCount * blob1.avgX
					+ blob2.correctPixelCount * blob2.avgX)
			/ (blob1.correctPixelCount + blob2.correctPixelCount);
	beacon->imageCenterY = float(
			blob1.correctPixelCount * blob1.avgY
					+ blob2.correctPixelCount * blob2.avgY)
			/ (blob1.correctPixelCount + blob2.correctPixelCount);
	if (camera_ == Camera::TOP)
		beacon->fromTopCamera = true;
	else
		beacon->fromTopCamera = false;

	std::cout << beacon->width << ", " << beacon->height << ","
			<< beacon->imageCenterX << "," << imageCenterY << std::endl;
}

void BeaconDetector::detectBeacon() {
	WorldObject* beacon_p_y =
			&vblocks_.world_object->objects_[WO_BEACON_PINK_YELLOW];
	WorldObject* beacon_y_p =
			&vblocks_.world_object->objects_[WO_BEACON_YELLOW_PINK];
	WorldObject* beacon_b_y =
			&vblocks_.world_object->objects_[WO_BEACON_BLUE_YELLOW];
	WorldObject* beacon_y_b =
			&vblocks_.world_object->objects_[WO_BEACON_YELLOW_BLUE];
	WorldObject* beacon_p_b =
			&vblocks_.world_object->objects_[WO_BEACON_PINK_BLUE];
	WorldObject* beacon_b_p =
			&vblocks_.world_object->objects_[WO_BEACON_BLUE_PINK];

	Blob pinkBlob, yellowBlob, blueBlob;

	for (unsigned int i = 0; i < blob_detector_->horizontalBlob[c_PINK].size();
			++i) {
		pinkBlob = blob_detector_->horizontalBlob[c_PINK][i];
		//Detect PINK_YELLOW
		for (unsigned int j = 0;
				j < blob_detector_->horizontalBlob[c_YELLOW].size(); ++j) {
			if (isVerticalConnected(pinkBlob, yellowBlob)) {
				bool flag = true;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (isVerticalConnected(yellowBlob,
							blob_detector_->horizontalBlob[c_PINK][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_BLUE].size();
						++ii) {
					if (isVerticalConnected(yellowBlob,
							blob_detector_->horizontalBlob[c_BLUE][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_YELLOW].size();
						++ii) {
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_YELLOW][ii],
							pinkBlob)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_BLUE].size();
						++ii) {
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_BLUE][ii],
							pinkBlob)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				if (flag == true) {
					formBeacon(beacon_p_y, pinkBlob, yellowBlob);
				}
			}
		}
	}
}
