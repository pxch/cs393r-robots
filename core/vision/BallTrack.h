#ifndef BALLTRACK_H
#define BALLTRACK_H

#include <common/WorldObject.h>
#include <vision/CameraMatrix.h>

#include <Eigen/Core>

/*
 * uses notation from http://robots.stanford.edu/probabilistic-robotics/ppt/kalman.ppt
 */
class BallTracker {
public:
	BallTracker() :
			seen(false) {

		for (int i = 0; i != 4; ++i) {
			for (int j = 0; j != 4; ++j) {
				A(i, j) = 0.0;
			}
		}
		for (int i = 0; i != 4; ++i) {
			A(i, i) = 1.0;
		}
		A(0, 2) = 1.0;
		A(1, 3) = 1.0;

		for (int i = 0; i != 4; ++i) {
			for (int j = 0; j != 4; ++j) {
				cov(i, j) = 0.0;
			}
		}
		for (int i = 0; i != 4; ++i) {
			cov(i, i) = 0.5;
		}

		for (int i = 0; i != 4; ++i) {
			R(i, i) = 0.5;
			Q(i, i) = 0.01;
		}

	}

	~BallTracker() {
	}

	/*
	 * x
	 * y
	 * v_x
	 * v_y
	 */
	Eigen::Vector4f state;

	Eigen::Matrix4f cov;

	void initState(float x, float y, float v_x, float v_y);

	/* called in the top camera */
	void track(WorldObject* ball, CameraMatrix &cmatrix_);

private:

	Eigen::Matrix4f R, Q;

	Eigen::Matrix4f A;

	bool seen;

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

};

#endif /* BALLTRACK_H */

