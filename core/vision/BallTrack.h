#ifndef BALLTRACK_H
#define BALLTRACK_H

#include <Eigen/Dense>

class BallTracker {
public:
	BallTracker(float dT_) :
			tracking(false), dT(dT_) {

		for (int i = 0; i != 4; ++i) {
			for (int j = 0; j != 4; ++j) {
				A(i, j) = 0.0;
			}
		}
		for (int i = 0; i != 4; ++i) {
			A(i, i) = 1.0;
		}
		A(0, 2) = dT_;
		A(1, 3) = dT_;

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

	Eigen::Matrix4f A;

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

