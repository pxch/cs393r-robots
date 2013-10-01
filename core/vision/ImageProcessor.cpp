#include <iostream>

#include "ImageProcessor.h"
#include "BallDetector.h"

ImageProcessor::ImageProcessor(VisionBlocks& vblocks,
		const ImageParams& iparams, Camera::Type camera) :
		vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(
				iparams_, camera), calibration_(NULL) {
	enableCalibration_ = false;
	classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
	blob_detector_ = new BlobDetector(DETECTOR_PASS_ARGS, classifier_);
	line_detector_ = new LineDetector(DETECTOR_PASS_ARGS, classifier_,
			blob_detector_);
	goal_detector_ = new GoalDetector(DETECTOR_PASS_ARGS, classifier_,
			blob_detector_, line_detector_);
	ball_detector_ = new BallDetector(DETECTOR_PASS_ARGS, classifier_,
			blob_detector_);
	robot_detector_ = new RobotDetector(DETECTOR_PASS_ARGS, classifier_,
			blob_detector_);
	cross_detector_ = new CrossDetector(DETECTOR_PASS_ARGS, classifier_,
			blob_detector_);
}

void ImageProcessor::init(TextLogger* tl) {
	textlogger = tl;
	vparams_.init();
	classifier_->init(tl);
	blob_detector_->init(tl);
	line_detector_->init(tl);
	goal_detector_->init(tl);
	ball_detector_->init(tl);
	robot_detector_->init(tl);
	cross_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
	if (camera_ == Camera::TOP)
		return vblocks_.image->getImgTop();
	return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg() {
	if (camera_ == Camera::TOP)
		return vblocks_.robot_vision->getSegImgTop();
	return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable() {
	return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix() {
	return cmatrix_;
}

void ImageProcessor::updateTransform() {
	BodyPart::Part camera;
	if (camera_ == Camera::TOP)
		camera = BodyPart::top_camera;
	else
		camera = BodyPart::bottom_camera;
	if (enableCalibration_) {
		float joints[NUM_JOINTS], sensors[NUM_SENSORS],
				dimensions[RobotDimensions::NUM_DIMENSIONS];
		memcpy(joints, vblocks_.joint->values_, NUM_JOINTS * sizeof(float));
		memcpy(sensors, vblocks_.sensor->values_, NUM_SENSORS * sizeof(float));
		memcpy(dimensions, vblocks_.robot_info->dimensions_.values_,
				RobotDimensions::NUM_DIMENSIONS * sizeof(float));
		Pose3D rel_parts[BodyPart::NUM_PARTS], abs_parts[BodyPart::NUM_PARTS];
		calibration_->applyJoints(joints);
		calibration_->applySensors(sensors);
		calibration_->applyDimensions(dimensions);
		ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
		Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
		ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
		ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
		cmatrix_.setCalibration(*calibration_);
		cmatrix_.updateCameraPose(abs_parts[camera]);
	} else {
		cmatrix_.updateCameraPose(vblocks_.body_model->abs_parts_[camera]);
	}
}

bool ImageProcessor::isRawImageLoaded() {
	if (camera_ == Camera::TOP)
		return vblocks_.image->img_top_;
	return vblocks_.image->img_bottom_;
}

int ImageProcessor::getImageHeight() {
	return iparams_.height;
}

int ImageProcessor::getImageWidth() {
	return iparams_.width;
}

void ImageProcessor::setCalibration(RobotCalibration calibration) {
	if (calibration_)
		delete calibration_;
	calibration_ = new RobotCalibration(calibration);
}

void ImageProcessor::ballInGoal() {
	WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];
	WorldObject* goal = &vblocks_.world_object->objects_[WO_OPP_GOAL];

	ball->type = 0;

	if (!ball->seen || !goal->seen) {
		ball->type = 0;
		return;
	}

	if (ball->fromTopCamera != goal->fromTopCamera) {
		return;
	}

	int total = 0;
	int good = 0; // pixels in goal or ball

	int x0 = ball->imageCenterX;
	int x1 = ball->imageCenterX;

	if (x0 == x1) {

		int y0, y1;
		if (ball->imageCenterY > goal->imageCenterY) {
			y0 = goal->imageCenterY;
			y1 = ball->imageCenterY;
		} else {
			y0 = ball->imageCenterX;
			y1 = goal->imageCenterY;
		}

		while (y0 <= y1) {
			++total;
			if (getSegPixelValueAt(x0,y0) == c_BLUE
					|| getSegPixelValueAt(x0,y0) == c_ORANGE) {
				++good;
			}
			++y0;
		}

	} else {

		int xStep = (x1 > x0) ? 1 : -1;

		int segCounts = (x1 > x0) ? x1 - x0 + 1 : x0 - x1 + 1;

		int y0 = ball->imageCenterY;
		int y1 = goal->imageCenterY;

		if (y0 == y1) {

			while (x0 != x1 + xStep) {
				if (getSegPixelValueAt(x0,y0) == c_BLUE
						|| getSegPixelValueAt(x0,y0) == c_ORANGE) {
					++good;
				}
				x0 += xStep;
				++total;
			}

		} else {
			x0 = goal->imageCenterX;
			y0 = goal->imageCenterY;
			x1 = ball->imageCenterX;
			y1 = ball->imageCenterY;
		}

		int yDist = 0;
		if (y0 != y1) {
			yDist = (y1 > y0) ? y1 - y0 + 1 : y0 - y1 + 1;
		}

		int yStep = 0;
		if (y0 != y1) {
			yStep = (y1 > y1) ? 1 : -1;
		}

		int ySegLen = yDist / segCounts;

		while (x0 != x1 + xStep) {
			x0 += xStep;

			for (int yCount = 0; yCount == ySegLen || y0 == y1 + yStep;
					++yCount) {
				if (getSegPixelValueAt(x0,y0) == c_BLUE
						|| getSegPixelValueAt(x0,y0) == c_ORANGE) {
					++good;
				}
				++total;
				y0 += yStep;
			}
		}
	}

	if (total == 0) {
		ball->type = 0;
		return;
	}

	if (float(good) / float(total) >= 0.8) {
		ball->type = 1;
	}

	return;
}

void ImageProcessor::processFrame() {
	updateTransform();
	classifier_->classifyImage(color_table_);

	ball_detector_->detectBall();
	goal_detector_->detectGoal();

	if (camera_ == Camera::TOP) {
		ballInGoal();
	}

	if (camera_ == Camera::BOTTOM) {
		getGroundLines();
	}
}

void ImageProcessor::getGroundLines() {
	int total = 0;
	bool bottomWhite = false;
	for (int x = 0; x != 320; ++x) {
		for (int y = 200; y != 210; ++y) {
			if (getSegPixelValueAt(x, y) == c_WHITE) {
				bottomWhite = true;
				++total;
			}
		}
	}
//	bool leftWhite = false;
//	for (int x = 0; x != 40; ++x) {
//		for (int y = 200; y != 220; ++y) {
//			if (getSegPixelValueAt(x, y) == c_WHITE) {
//				centerWhite = true;
//				++total;
//			}
//		}
//	}
//	bool rightWhite = false;
//	for (int x = 280; x != 280; ++x) {
//		for (int y = 200; y != 220; ++y) {
//			if (getSegPixelValueAt(x, y) == c_WHITE) {
//				centerWhite = true;
//				++total;
//			}
//		}
//	}
	bool topWhite = false;
	for (int x = 0; x != 320; ++x) {
		for (int y = 0; y != 10; ++ y) {
			if (getSegPixelValueAt(x,y) == c_WHITE) {
				topWhite = true;
				++ total;
			}
		}
	}
	WorldObject *line = &vblocks_.world_object->objects_[WO_OPP_GOAL_LINE];
	line->fieldLineIndex = 0;
	if (bottomWhite) {
		line->fieldLineIndex += 1;
	}
	if (topWhite) {
		line->fieldLineIndex += 2;
	}
	line->pixelCount = total;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
	color_table_ = table;
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
	std::vector<BallCandidate*> candidates;
	return candidates;
}

BallCandidate * ImageProcessor::getBestBallCandidate() {
	return NULL;
}

void ImageProcessor::enableCalibration(bool value) {
	enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
	return vblocks_.image->loaded_;
}
