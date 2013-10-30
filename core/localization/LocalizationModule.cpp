#include <localization/LocalizationModule.h>
#include <iostream>

void LocalizationModule::specifyMemoryDependency() {
	requiresMemoryBlock("world_objects");
	requiresMemoryBlock("localization");
	requiresMemoryBlock("team_packets");
	requiresMemoryBlock("vision_frame_info");
	requiresMemoryBlock("vision_odometry");
	requiresMemoryBlock("robot_state");
	requiresMemoryBlock("game_state");
	requiresMemoryBlock("vision_joint_angles");
	requiresMemoryBlock("behavior");
	requiresMemoryBlock("vision_processed_sonar");
	requiresMemoryBlock("delayed_localization");
}

void LocalizationModule::specifyMemoryBlocks() {
	getOrAddMemoryBlock(worldObjects, "world_objects");
	getOrAddMemoryBlock(localizationMem, "localization");
	getOrAddMemoryBlock(teamPacketsMem, "team_packets");
	getOrAddMemoryBlock(frameInfo, "vision_frame_info");
	getOrAddMemoryBlock(odometry, "vision_odometry");
	getOrAddMemoryBlock(robotState, "robot_state");
	getOrAddMemoryBlock(gameState, "game_state");
	getOrAddMemoryBlock(jointAngles, "vision_joint_angles");
	getOrAddMemoryBlock(behaviorMem, "behavior");
	getOrAddMemoryBlock(processedSonar, "vision_processed_sonar");
	getOrAddMemoryBlock(delayedLocalization, "delayed_localization");
}

void LocalizationModule::initSpecificModule() {
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.loc.x = 0;
		p.loc.y = 0;
		p.theta = 0;
		p.prob = 1.0;
	}
	copyParticles();

	resetParticles();
	for (int i = 0; i < NUM_PARTICLES; i++) {
		std::cout << "Particle " << i << ": " << particles_[i].loc.x << ", "
				<< particles_[i].loc.y << std::endl;
	}
	std::cout << "Reset Particles..." << std::endl;

}

void LocalizationModule::processFrame() {
	int frameID = frameInfo->frame_id;
	std::cout << "Frame: " << frameID << std::endl;

	// 1. Update particles from observations
	updateParticlesFromOdometry();
	updateParticlesFromSensor();

	// 2. If this is a resampling frame, resample
//	if (frameID % RESAMPLE_FREQ == 0)
		resamplingParticles();

	// 3. Update the robot's pose
	updatePose();

	// 4. If this is a random walk frame, random walk
//	if (frameID % RANDOM_WALK_FREQ == 0)
		randomWalkParticles();

	// 5. Copy particles to localization memory:
	copyParticles();
}

void LocalizationModule::updateParticlesFromSensor() {
	std::cout << "Update Particles From Sensor..." << std::endl;

	WorldObject* beacon_p_y = &worldObjects->objects_[WO_BEACON_PINK_YELLOW];
	WorldObject* beacon_y_p = &worldObjects->objects_[WO_BEACON_YELLOW_PINK];
	WorldObject* beacon_b_y = &worldObjects->objects_[WO_BEACON_BLUE_YELLOW];
	WorldObject* beacon_y_b = &worldObjects->objects_[WO_BEACON_YELLOW_BLUE];
	WorldObject* beacon_p_b = &worldObjects->objects_[WO_BEACON_PINK_BLUE];
	WorldObject* beacon_b_p = &worldObjects->objects_[WO_BEACON_BLUE_PINK];

	if (beacon_p_y->seen)
		updateParticlesFromBeacon(beacon_p_y);
	if (beacon_y_p->seen)
		updateParticlesFromBeacon(beacon_y_p);
	if (beacon_b_y->seen)
		updateParticlesFromBeacon(beacon_b_y);
	if (beacon_y_b->seen)
		updateParticlesFromBeacon(beacon_y_b);
	if (beacon_p_b->seen)
		updateParticlesFromBeacon(beacon_p_b);
	if (beacon_b_p->seen)
		updateParticlesFromBeacon(beacon_b_p);
}

void LocalizationModule::updateParticlesFromBeacon(WorldObject* beacon) {

	float prob_seq[NUM_PARTICLES] = { 0.0 };
	float prob_mean = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		prob_seq[i] = particles_[i].prob;
		prob_mean += prob_seq[i];
	}
	prob_mean /= NUM_PARTICLES;
	float prob_se = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		prob_se += (prob_seq[i] - prob_mean) * (prob_seq[i] - prob_mean);
	}
	prob_se /= NUM_PARTICLES;
	prob_se = sqrt(prob_se);

	float normalization = prob_se;

	float distanceBias = 0.0;
	float bearingBias = 0.0;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		distanceBias = abs(
				beacon->visionDistance - p.loc.getDistanceTo(beacon->loc));
		bearingBias = abs(
				beacon->visionBearing
						- p.loc.getBearingTo(beacon->loc, p.theta));

		float prob_multiplier = exp(
				-0.5 * (distanceBias * distanceBias + bearingBias * bearingBias)
						/ normalization);
		p.prob = p.prob * prob_multiplier;
	}
}

void LocalizationModule::resamplingParticles() {
	std::cout << "Resampling Particles..." << std::endl;
	float sumProb = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		previous_particles_[i] = particles_[i];
		sumProb += particles_[i].prob;
	}
	for (int i = 0; i < NUM_PARTICLES; i++) {
		previous_particles_[i].prob /= sumProb;
	}

	int current_index = 0;
	int previous_index = -1;
	while (current_index < NUM_PARTICLES) {
		int previous_index = sampleIndexFromRandom(drand48());
		if (previous_index == -1)
			continue;
		else {
			particles_[current_index] = previous_particles_[previous_index];
			particles_[current_index].prob = 1.0;
			current_index++;
		}
	}
}

int LocalizationModule::sampleIndexFromRandom(float random) {
	if (random < 0.0 || random > 1.0)
		return -1;
	float sumProb = 0.0;
	int index = 0;
	while (sumProb < random && index < NUM_PARTICLES) {
		sumProb += previous_particles_[index].prob;
		index++;
	}
	if (index == NUM_PARTICLES)
		return index - 1;
	else
		return index;
}

void LocalizationModule::copyParticles() {
	memcpy(localizationMem->particles, particles_,
			NUM_PARTICLES * sizeof(Particle));
}

void LocalizationModule::updateParticlesFromOdometry() {

	std::cout << "Update Particles From Odometry..." << std::endl;

	Pose2D disp = odometry->displacement;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.moveRelative(disp);
		p.degradeProbability(DEGRADE_FACTOR);
	}
}

void LocalizationModule::resetParticles() {
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.prob = 1.0f;
		p.placeRandomly();
	}
}

void LocalizationModule::setParticleProbabilities(float newProb) {
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.prob = newProb;
	}
}

void LocalizationModule::randomWalkParticles() {
	// loop through half, moving each pair of particles opposite directions

	for (int i = 0; i < NUM_PARTICLES / 2; i++) {
		Particle& part1 = particles_[i];
		Particle& part2 = particles_[i + NUM_PARTICLES / 2];

		Vector2D dPos(DELTA_DIST * (2.0 * drand48() - 1),
				DELTA_DIST * (2.0 * drand48() - 1));
		AngRad dAng = DELTA_ANG * (2.0 * drand48() - 1);

		// move them in opposite directions on this vector, based on their prob
		float p1Ratio = 1.0 - part1.prob;
		float p2Ratio = 1.0 - part2.prob;

		float p1AngleRatio = p1Ratio;
		float p2AngleRatio = p2Ratio;

		part1.move(dPos * p1Ratio, p1AngleRatio * dAng);
		part2.move(-dPos * p2Ratio, p2AngleRatio * -dAng);

	}
}

void LocalizationModule::updatePose() {
	WorldObject& self = worldObjects->objects_[robotState->WO_SELF];
	// Compute a weighted average of the particles to fill in your location

	Point2D robotLoc(0.0, 0.0);
	AngRad robotOrient = 0.0;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		robotLoc += p.loc * p.prob;
		robotOrient += p.theta * p.prob;
	}

	robotLoc /= NUM_PARTICLES;
	robotOrient /= NUM_PARTICLES;

	self.loc = robotLoc;
	self.orientation = robotOrient;

	std::cout << "Robot Location: " << robotLoc.x << ", " << robotLoc.y
			<< ", Robot Orientation: " << robotOrient << std::endl;
}
