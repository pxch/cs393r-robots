#include <vision/BallTrack.h>

void BallTracker::initState(float x, float y, float v_x, float v_y) {
	state(0) = x;
	state(1) = y;
	state(2) = v_x;
	state(3) = v_y;
}

/* observed values */
void BallTracker::updateState(float x, float y, float v_x, float v_y) {

	Eigen::Matrix4f R, Q;
	for (int i = 0; i != 4; ++i) {
		R(i, i) = 0.5;
		Q(i, i) = 0.5;
	}

	Eigen::Vector4d state_ = A * state;
	Eigen::Matrix4f cov_ = A * cov * A.transpose() + R;

	Eigen::Matrix4f tmp1 = cov_ + Q;
	Eigen::Matrix4f K = cov_ * tmp1.inverse();

	Eigen::Vector4d obs(x, y, v_x, v_y);

	state = state_ + K * (obs - state_);
	cov = (Eigen::Matrix4f::Identity() - K) * cov_;

}
