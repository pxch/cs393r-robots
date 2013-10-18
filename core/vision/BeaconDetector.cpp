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
	float horizontal_inter_thres = 0.5;

	int vertical_dist = 0;
	float horizontal_inter = 0.0;

	if (float(blob1.dx + blob1.dy) / float(blob2.dx + blob2.dy) > 1.25
			|| float(blob1.dx + blob1.dy) / float(blob2.dx + blob2.dy) < 0.8)
		return false;

	vertical_dist =
			blob1.yf > blob2.yi ? blob1.yf - blob2.yi : blob2.yi - blob1.yf;
	if (blob1.xf <= blob2.xi || blob2.xf <= blob1.xi)
		return false;
	if (abs(int(blob1.xi) - int(blob2.xf))
			> abs(int(blob1.xf) - int(blob2.xi))) {
		horizontal_inter = float(abs(int(blob1.xf) - int(blob2.xi)))
				/ float(blob1.dx);
	} else {
		horizontal_inter = float(abs(int(blob1.xi) - int(blob2.xf)))
				/ float(blob1.dx);
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
	beacon->imageCenterX = (float(blob1.xi) + float(blob2.xf)) / 2;
	beacon->imageCenterY = (float(blob1.yi) + float(blob2.yf)) / 2;
	if (camera_ == Camera::TOP)
		beacon->fromTopCamera = true;
	else
		beacon->fromTopCamera = false;

	std::cout << "Width: " << beacon->width << ", Height: " << beacon->height
			<< ", imageCenterX: " << beacon->imageCenterX << ", imageCenterY: "
			<< beacon->imageCenterY << std::endl;
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

	detectBeacon(beacon_p_y, c_PINK, c_YELLOW, "PINK", "YELLOW");
	detectBeacon(beacon_y_p, c_YELLOW, c_PINK, "YELLOW", "PINK");
	detectBeacon(beacon_b_y, c_BLUE, c_YELLOW, "BLUE", "YELLOW");
	detectBeacon(beacon_y_b, c_YELLOW, c_BLUE, "YELLOW", "BLUE");
	detectBeacon(beacon_p_b, c_PINK, c_BLUE, "PINK", "BLUE");
	detectBeacon(beacon_b_p, c_BLUE, c_PINK, "BLUE", "PINK");
}

void BeaconDetector::detectBeacon(WorldObject* beacon, int color1, int color2,
		std::string color1_str, std::string color2_str) {
	Blob blob1, blob2;
	for (unsigned int i = 0; i < blob_detector_->horizontalBlob[color1].size();
			++i) {
		blob1 = blob_detector_->horizontalBlob[color1][i];

		for (unsigned int j = 0;
				j < blob_detector_->horizontalBlob[color2].size(); ++j) {
			blob2 = blob_detector_->horizontalBlob[color2][j];

			if (isVerticalConnected(blob1, blob2)) {
//				std::cout << "------------------------------" << std::endl;
//				std::cout << "Beacon_" << color1_str << "_" << color2_str << "connected. Waiting for further check."<< std::endl;
//				std::cout << "------------------------------" << std::endl;

				bool flag = true;

				//Check blob2
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (color2 == c_PINK && j == ii)
						continue;
					if (isVerticalConnected(blob2,
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
					if (color2 == c_BLUE && j == ii)
						continue;
					if (isVerticalConnected(blob2,
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
					if (color2 == c_YELLOW && j == ii)
						continue;
					if (isVerticalConnected(blob2,
							blob_detector_->horizontalBlob[c_YELLOW][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;

				//Check blob1
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (color1 == c_PINK && i == ii)
						continue;
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_PINK][ii],
							blob1)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_BLUE].size();
						++ii) {
					if (color1 == c_BLUE && i == ii)
						continue;
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_BLUE][ii],
							blob1)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_YELLOW].size();
						++ii) {
					if (color1 == c_YELLOW && i == ii)
						continue;
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_YELLOW][ii],
							blob1)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;

				//Pass all check, find beacon
				if (flag == true) {
					std::cout << "------------------------------" << std::endl;
					std::cout << "Find Beacon_" << color1_str << "_" << color2_str << std::endl;
					formBeacon(beacon, blob1, blob2);
					std::cout << "------------------------------" << std::endl;
				}
			}
		}
	}
}

