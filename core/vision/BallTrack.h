#ifndef BALLTRACK_H
#define BALLTRACK_H

#include <Eigen/Dense>

class BallTracker {
public:
	BallTracker(float dT_) :
			tracking(false), dT(dT_) {
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

	float const dT; /* time interval between 2 observations */

	void initState(float x, float y, float v_x, float v_y);

	/*
	 * The model:
	 *
	 * X = [ position \\
	 * 		 velocity ]
	 *
	 * X_{n+1} = ( 1 delta_T \\ X_n
	 *             0       1 )
	 */
	void updateState(float x, float y, float v_x, float v_y); /* observed values */

private:

};

#endif /* BALLTRACK_H */

