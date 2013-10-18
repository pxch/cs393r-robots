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

	if (float(blob1.dx + blob1.dy) / float(blob2.dx + blob2.dy) > 1.25 || float(blob1.dx + blob1.dy) / float(blob2.dx + blob2.dy) < 0.8)
		return false;

	vertical_dist =
			blob1.yf > blob2.yi ? blob1.yf - blob2.yi : blob2.yi - blob1.yf;
	if (blob1.xf <= blob2.xi || blob2.xf <= blob1.xi)
		return false;
	if (abs(int(blob1.xi) - int(blob2.xf)) > abs(int(blob1.xf) - int(blob2.xi))) {
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

	Blob pinkBlob, yellowBlob, blueBlob;

	for (unsigned int i = 0; i < blob_detector_->horizontalBlob[c_PINK].size();
			++i) {
		pinkBlob = blob_detector_->horizontalBlob[c_PINK][i];
		//Detect PINK_YELLOW
		for (unsigned int j = 0;
				j < blob_detector_->horizontalBlob[c_YELLOW].size(); ++j) {
			yellowBlob = blob_detector_->horizontalBlob[c_YELLOW][j];
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
					std::cout << "------------------------------" << std::endl;
					std::cout << "Find Beacon_Pink_Yellow" << std::endl;
					formBeacon(beacon_p_y, pinkBlob, yellowBlob);
					std::cout << "------------------------------" << std::endl;
				}
			}
		}

		//Detect PINK_BLUE
		for (unsigned int j = 0;
				j < blob_detector_->horizontalBlob[c_BLUE].size(); ++j) {
			blueBlob = blob_detector_->horizontalBlob[c_BLUE][j];
			if (isVerticalConnected(pinkBlob, blueBlob)) {
				bool flag = true;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (isVerticalConnected(blueBlob,
							blob_detector_->horizontalBlob[c_PINK][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_YELLOW].size();
						++ii) {
					if (isVerticalConnected(blueBlob,
							blob_detector_->horizontalBlob[c_YELLOW][ii])) {
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
					std::cout << "------------------------------" << std::endl;
					std::cout << "Find Beacon_Pink_Blue" << std::endl;
					formBeacon(beacon_p_b, pinkBlob, blueBlob);
					std::cout << "------------------------------" << std::endl;
				}
			}
		}

	}

	for (unsigned int i = 0; i < blob_detector_->horizontalBlob[c_BLUE].size();
			++i) {
		blueBlob = blob_detector_->horizontalBlob[c_BLUE][i];
		//Detect BLUE_YELLOW
		for (unsigned int j = 0;
				j < blob_detector_->horizontalBlob[c_YELLOW].size(); ++j) {
			yellowBlob = blob_detector_->horizontalBlob[c_YELLOW][j];
			if (isVerticalConnected(blueBlob, yellowBlob)) {
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
							blueBlob)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_PINK][ii],
							blueBlob)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				if (flag == true) {
					std::cout << "------------------------------" << std::endl;
					std::cout << "Find Beacon_BLUE_Yellow" << std::endl;
					formBeacon(beacon_b_y, blueBlob, yellowBlob);
					std::cout << "------------------------------" << std::endl;
				}
			}
		}

		//Detect BLUE_PINK
		for (unsigned int j = 0;
				j < blob_detector_->horizontalBlob[c_PINK].size(); ++j) {
			pinkBlob = blob_detector_->horizontalBlob[c_PINK][j];
			if (isVerticalConnected(blueBlob, pinkBlob)) {
				bool flag = true;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_YELLOW].size();
						++ii) {
					if (isVerticalConnected(pinkBlob,
							blob_detector_->horizontalBlob[c_YELLOW][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_BLUE].size();
						++ii) {
					if (isVerticalConnected(pinkBlob,
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
							blueBlob)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_PINK][ii],
							blueBlob)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				if (flag == true) {
					std::cout << "------------------------------" << std::endl;
					std::cout << "Find Beacon_BLUE_PINK" << std::endl;
					formBeacon(beacon_b_p, blueBlob, pinkBlob);
					std::cout << "------------------------------" << std::endl;
				}
			}
		}
	}

	for (unsigned int i = 0; i < blob_detector_->horizontalBlob[c_YELLOW].size();
			++i) {
		yellowBlob = blob_detector_->horizontalBlob[c_YELLOW][i];
		//Detect YELLOW_PINK
		for (unsigned int j = 0;
				j < blob_detector_->horizontalBlob[c_PINK].size(); ++j) {
			pinkBlob = blob_detector_->horizontalBlob[c_PINK][j];
			if (isVerticalConnected(yellowBlob, pinkBlob)) {
				bool flag = true;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_YELLOW].size();
						++ii) {
					if (isVerticalConnected(pinkBlob,
							blob_detector_->horizontalBlob[c_YELLOW][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_BLUE].size();
						++ii) {
					if (isVerticalConnected(pinkBlob,
							blob_detector_->horizontalBlob[c_BLUE][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_PINK][ii],
							yellowBlob)) {
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
							yellowBlob)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				if (flag == true) {
					std::cout << "------------------------------" << std::endl;
					std::cout << "Find Beacon_YELLOW_PINK" << std::endl;
					formBeacon(beacon_y_p, yellowBlob, pinkBlob);
					std::cout << "------------------------------" << std::endl;
				}
			}
		}

		//Detect YELLOW_BLUE
		for (unsigned int j = 0;
				j < blob_detector_->horizontalBlob[c_BLUE].size(); ++j) {
			blueBlob = blob_detector_->horizontalBlob[c_BLUE][j];
			if (isVerticalConnected(yellowBlob, blueBlob)) {
				bool flag = true;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_YELLOW].size();
						++ii) {
					if (isVerticalConnected(blueBlob,
							blob_detector_->horizontalBlob[c_YELLOW][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (isVerticalConnected(blueBlob,
							blob_detector_->horizontalBlob[c_PINK][ii])) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				for (unsigned int ii = 0;
						ii < blob_detector_->horizontalBlob[c_PINK].size();
						++ii) {
					if (isVerticalConnected(
							blob_detector_->horizontalBlob[c_PINK][ii],
							yellowBlob)) {
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
							yellowBlob)) {
						flag = false;
						break;
					}
				}
				if (flag == false)
					break;
				if (flag == true) {
					std::cout << "------------------------------" << std::endl;
					std::cout << "Find Beacon_YELLOW_BLUE" << std::endl;
					formBeacon(beacon_y_b, yellowBlob, blueBlob);
					std::cout << "------------------------------" << std::endl;
				}
			}
		}
	}
}
