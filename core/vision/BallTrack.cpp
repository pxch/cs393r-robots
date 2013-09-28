#include <cstdio>

#include <vision/BallTrack.h>

#include <Eigen/LU>

void BallTracker::initState(float x, float y, float v_x, float v_y) {
	state(0) = x;
	state(1) = y;
	state(2) = v_x;
	state(3) = v_y;
}

/* observed values */
void BallTracker::updateState(float x, float y) {

	Eigen::Vector4f state_ = A * state;
	Eigen::Matrix4f cov_ = A * cov * A.transpose() + R;

	Eigen::Matrix2f tmp1 = C * cov_ * C.transpose() + Q;
	Eigen::Matrix<float, 4, 2> K = cov_ * C.transpose() * tmp1.inverse();

	Eigen::Vector2f obs(x, y);

	state = state_ + K * (obs - C * state_);
	cov = (Eigen::Matrix4f::Identity() - K * C) * cov_;

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
		updateState(p.x, p.y);
	}

	if (ball->seen) {
		printf("camera_pos %f %f kalman_pos %f %f kalman_vel %f %f \n", p.x,
				p.y, state(0), state(1), state(2), state(3));
	}
}
