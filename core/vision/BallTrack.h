#ifndef BALLTRACK_H
#define BALLTRACK_H

#include <Eigen/Core>
#include <Eigen/LU>

class BallTracker {
public:
	BallTracker() :
			tracking(false) {
	}

	~BallTracker() {
	}

	bool tracking;

private:

};

#endif /* BALLTRACK_H */

