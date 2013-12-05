#include "BallDetector.h"
#include <iostream>

using namespace Eigen;

#define getball() (&vblocks_.world_object->objects_[WO_BALL])
#define getself() (&vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF])
#define getframe() vblocks_.frame_info->frame_id

#define BALL_REL_X_OFFSET -70

BallDetector::BallDetector(DETECTOR_DECLARE_ARGS, Classifier*& classifier, BlobDetector*& blob_detector) : DETECTOR_INITIALIZE, classifier_(classifier), blob_detector_(blob_detector) {
	candidateCount = 0;
}

void BallDetector::detectBall() {
	WorldObject* ball = &vblocks_.world_object->objects_[WO_BALL];

	Blob ball_blob;
	Blob orange_blob;
	
	bool flag = false;
	for (int i = 0; i < blob_detector_->horizontalBlob[c_ORANGE].size(); ++i) {
		orange_blob = blob_detector_->horizontalBlob[c_ORANGE][i];
		if (orange_blob.dx + orange_blob.dy > ball_blob.dx + ball_blob.dy) {
			if (float(orange_blob.dx) / orange_blob.dy < 1.25  && float(orange_blob.dx) / orange_blob.dy > 0.8) {
				ball_blob = orange_blob;
				flag = true;
			}
		}
	}

	if (ball_blob.dx == 0 || ball_blob.dy == 0 || !flag) {
		seen = false;
		ball->seen = false;
		ball->imageCenterX = 0;
		ball->imageCenterY = 0;
		ball->relPos.x = 0;
		ball->relPos.y = 0;
		ball->radius = 0;
	}
	else {
		seen = true;
		ball->seen = true;
		int imageX = (ball_blob.xi + ball_blob.xf) / 2;
		int imageY = (ball_blob.yi + ball_blob.yf) / 2;
		ball->imageCenterX = imageX;
		ball->imageCenterY = imageY;
		ball->radius = (ball_blob.dx + ball_blob.dy) / 4;

		Position p = cmatrix_.getWorldPosition(imageX, imageY);
		ball->visionBearing = cmatrix_.bearing(p);
		ball->visionElevation = cmatrix_.elevation(p);
		ball->visionDistance = cmatrix_.groundDistance(p);

		BallCandidate* candidate = &candidates[0];

		candidate->centerX = imageX;
		candidate->centerY = imageY;
		candidate->radius = (ball_blob.dx + ball_blob.dy) / 4;
		candidate->width = ball_blob.dx;
		candidate->height = ball_blob.dy;

		float directDistance = getDirectDistance(candidate);
		directDistance = cmatrix_.getWorldDistanceByWidth(candidate->width, BALL_RADIUS * 2);
		candidate->relPosition = cmatrix_.getWorldPositionByDirectDistance(candidate->centerX, candidate->centerY, directDistance);
		//////
		candidate->relPosition.x = candidate->relPosition.x + BALL_REL_X_OFFSET;
		//////
		candidate->groundDistance = cmatrix_.groundDistance(candidate->relPosition);
		candidate->relPosition.z -= BALL_RADIUS;
		candidate->valid = true;

                ball->distance = candidate->groundDistance;
		ball->relPos.x = candidate->relPosition.x;
		ball->relPos.y = candidate->relPosition.y;

		std::cout << "Ball CenterX: " << ball->imageCenterX << ", CenterY: " << ball->imageCenterY << ", RelativeX: " << ball->relPos.x << ", RelativeY: " << ball->relPos.y << std::endl;
	}
}

bool BallDetector::bestCandidateFound() {
	return seen;
}

float BallDetector::getDirectDistanceByBlobWidth(float dx, int /*imageX*/, int /*imageY*/) {
	dx *= 640.0f / iparams_.width; // Normalize because this was tuned with a width of 640
	//float dist = 24269.169211 * powf(dx,-0.904299);  // tuned for exploreUT13
	float dist = 28699.871512 * powf(dx,-0.923806); // tuned on 6/14/13 by sbarrett and katie
	//printf("\tBall distance for diameter(%0.3f): %0.3f\n",dx,dist);
	return dist;
}

float BallDetector::getDirectDistanceByKinematics(int x, int y) {
	Position p = cmatrix_.getWorldPosition(x, y, BALL_RADIUS);
	float dist = cmatrix_.directDistance(p);
	return dist;
}

float BallDetector::getDirectDistance(BallCandidate* candidate) {
	float dist;
	float wdist = getDirectDistanceByBlobWidth(2.0f*candidate->radius, candidate->centerX, candidate->centerY);
	float kdist = getDirectDistanceByKinematics(candidate->centerX, candidate->centerY);

	// Kdist is better up close, wdist is better farther away. We scale the ratio of each so that we
	// don't get large discrepancies at the cutoff points
	float minKdist = 1000, maxKdist = 10000;
	float wdistRatio = pow((kdist - minKdist) / (maxKdist - minKdist), 2);
	if(kdist > maxKdist) dist = wdist;
	else if (kdist > minKdist) dist = kdist * (1 - wdistRatio) + wdist * wdistRatio;
	else dist = kdist;

	candidate->kwDistanceDiscrepancy = fabs(kdist - wdist) / (kdist + wdist);

//	dist = wdist;
	return dist;
}

