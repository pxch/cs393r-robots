#include <cstdio>

#include <vision/BallTrack.h>

#include <Eigen/LU>

void BallTracker::initState(float x, float y, float v_x, float v_y) {
	state(0) = x;
	state(1) = y;
	state(2) = v_x;
	state(3) = v_y;
	prev_x = x;
	prev_y = y;
}

/* observed values */
void BallTracker::updateState(float x, float y, float v_x, float v_y) {

	Eigen::Vector4f state_ = A * state;
	Eigen::Matrix4f cov_ = A * cov * A.transpose() + R;

	Eigen::Matrix4f tmp1 = cov_ + Q;
	Eigen::Matrix4f K = cov_ * tmp1.inverse();

	Eigen::Vector4f obs(x, y, v_x, v_y);

	state = state_ + K * (obs - state_);
	cov = (Eigen::Matrix4f::Identity() - K) * cov_;

}

void BallTracker::track(WorldObject* ball, CameraMatrix &cmatrix_) {
	if (!ball->seen) {
		seen = false;
		return;
	}

	Position p = cmatrix_.getWorldPosition(ball->imageCenterX,
			ball->imageCenterY);

	if (!seen) {
		seen = true;
		initState(p.x, p.y, 0.0, 0.0);
	} else {
		float v_x = p.x - prev_x;
		float v_y = p.y - prev_y;
		updateState(p.x, p.y, v_x, v_y);
	}
}
