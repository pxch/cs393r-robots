#include <vision/BallTrack.h>

void BallTracker::initState(float x, float y, float v_x, float v_y) {
	state(0) = x;
	state(1) = y;
	state(2) = v_x;
	state(3) = v_y;
}

