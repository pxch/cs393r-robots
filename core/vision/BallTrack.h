#ifndef BALLTRACK_H
#define BALLTRACK_H

#include <Eigen/Dense>

class BallTracker {
public:
	BallTracker() :
			tracking(false) {
	}

	~BallTracker() {
	}

	/*
	 * true - if ball is being tracked
	 */
	bool tracking;

	/*
	 * x
	 * y
	 * v_x
	 * v_y
	 */
	Eigen::Vector4d state;

	void initState(float x, float y, float v_x, float v_y);

private:

};

#endif /* BALLTRACK_H */

