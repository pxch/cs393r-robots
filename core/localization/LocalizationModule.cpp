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
	resetParticles();

	copyParticles();

	innerFrameIndex = 1;

	trappedInWrongPosition = false;
}

void LocalizationModule::processFrame() {
	int frameID = frameInfo->frame_id;
	std::cout << "Frame: " << innerFrameIndex << std::endl;

	if (trappedInWrongPosition) {
		resetParticles();
		trappedInWrongPosition = false;
	}

	// 1. Update particles from observations
	updateParticlesFromOdometry();
	updateParticlesFromSensor();

	// 2. If this is a resampling frame, resample
	if (innerFrameIndex % RESAMPLE_FREQ == 0)
		resamplingParticles2();

	// 3. Update the robot's pose
	updatePose();

	// 4. If this is a random walk frame, random walk
	if (innerFrameIndex % RANDOM_WALK_FREQ == 1)
		randomWalkParticles();

	// 5. Copy particles to localization memory:
	copyParticles();

	innerFrameIndex++;
}

void LocalizationModule::updateParticlesFromSensor() {
//	std::cout << "Updating Particles From Sensor..." << std::endl;

	// Reset Beacon Location in each Frame
	for (int i = NUM_LANDMARKS - 6; i < NUM_LANDMARKS; i++) {
		WorldObject *beacon = &worldObjects->objects_[i + LANDMARK_OFFSET];
		beacon->loc = landmarkLocation[i];
		if (beacon->seen)
			updateParticlesFromBeacon(beacon);
	}
}

void LocalizationModule::updateParticlesFromBeacon(WorldObject* beacon) {
	float degrade_factor = 0;

	float distanceBias[NUM_PARTICLES];
	float bearingBias[NUM_PARTICLES];

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		float particleDistance = p.loc.getDistanceTo(beacon->loc);
		float particleBearing = p.loc.getBearingTo(beacon->loc, p.theta);

		distanceBias[i] = abs(beacon->visionDistance - particleDistance);
		bearingBias[i] = abs(beacon->visionBearing - particleBearing);

		if (distanceBias[i] / beacon->visionDistance < 0.25) {
			if (bearingBias[i] < M_PI / 4)
				degrade_factor = 1;
			else if (bearingBias[i] < M_PI / 2)
				degrade_factor = 0.8;
			else
				degrade_factor = 0.5;
		} else if (distanceBias[i] / beacon->visionDistance < 0.5) {
			if (bearingBias[i] < M_PI / 4)
				degrade_factor = 0.8;
			else if (bearingBias[i] < M_PI / 2)
				degrade_factor = 0.6;
			else
				degrade_factor = 0.3;
		} else {
			if (bearingBias[i] < M_PI / 4)
				degrade_factor = 0.6;
			else if (bearingBias[i] < M_PI / 2)
				degrade_factor = 0.4;
			else
				degrade_factor = 0.1;
		}

		p.degradeProbability(degrade_factor);
	}

	float sumProb = 0.0;
	float mean_distanceBias = 0.0;
	float var_distanceBias = 0.0;
	Point2D mean_loc(0.0, 0.0);
	AngRad mean_theta = 0.0;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		sumProb += p.prob;
		mean_distanceBias += distanceBias[i] * p.prob;

		mean_loc += p.loc * p.prob;
		mean_theta += p.theta * p.prob;
	}

	mean_distanceBias /= sumProb;

	mean_loc /= sumProb;
	mean_theta /= sumProb;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
//		p.loc = p.loc + (mean_loc - p.loc) * (1 - p.prob) * 0.2;
//		p.theta = p.theta + (mean_theta - p.theta) * (1 - p.prob) * 0.2;
		
		p.loc = p.loc + (mean_loc - p.loc) * (1 - p.prob) * 0.25
			+ (beacon->loc - p.loc) * (p.loc.getDistanceTo(beacon->loc) / beacon->visionDistance - 1) * (1 - p.prob) * 0.25;
		p.theta = p.theta + (mean_theta - p.theta) * (1 - p.prob) * 0.25
			+ (p.loc.getBearingTo(beacon->loc, p.theta) - beacon->visionBearing) * (1 - p.prob) * 0.25;

//		p.theta = p.theta + (mean_theta - p.theta) * (1 - p.prob) * 0.25
//			+ (mean_loc.getBearingTo(beacon->loc, mean_theta) - beacon->visionBearing) * (1 - p.prob) * 0.25;

		var_distanceBias += (distanceBias[i] - mean_distanceBias)
			* (distanceBias[i] - mean_distanceBias) * p.prob;
	}

	var_distanceBias /= sumProb;


	if (var_distanceBias < 20 * 20 &&
		(mean_distanceBias / beacon->visionDistance > 0.5 || 
			mean_loc.getBearingTo(beacon->loc, mean_theta) > M_PI / 4))
		trappedInWrongPosition = true;
}

void LocalizationModule::resamplingParticles() {
	std::cout << "Resampling Particles..." << std::endl;

	float sumProb = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		sumProb += particles_[i].prob;
	}
	for (int i = 0; i < NUM_PARTICLES; i++) {
		particles_[i].prob /= (sumProb / NUM_PARTICLES);
	}

	for (int i = 0; i < NUM_PARTICLES; i++) {
		previous_particles_[i] = particles_[i];
	}

	int current_index = 0;
	int previous_index = -1;
	float random = 0.0;

	while (current_index < NUM_PARTICLES) {
		random = (float) NUM_PARTICLES * drand48();
		previous_index = sampleIndexFromRandom(random);
		if (previous_index == -1)
			continue;
		else {
//			std::cout << "(" << random << ", " << previous_index << "), ";
			particles_[current_index] = previous_particles_[previous_index];
			particles_[current_index].prob = 1.0f;
			current_index++;
		}
	}
//	std::cout << std::endl;

//	printParticles();
}

int LocalizationModule::sampleIndexFromRandom(float random) {
	if (random < 0.0 || random > NUM_PARTICLES)
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

void LocalizationModule::resamplingParticles2() {
	std::cout << "Resampling Particles..." << std::endl;

	float sumProb = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		sumProb += particles_[i].prob;
	}
	for (int i = 0; i < NUM_PARTICLES; i++) {
		particles_[i].prob /= (sumProb / NUM_PARTICLES);
	}

	for (int i = 0; i < NUM_PARTICLES; i++) {
		previous_particles_[i] = particles_[i];
	}
	
	int index = 0;
	float newWeightC[NUM_PARTICLES];
	float newWeightU[NUM_PARTICLES];

	newWeightC[0] = previous_particles_[0].prob;
	for (int i = 1; i < NUM_PARTICLES; i++)
		newWeightC[i] = newWeightC[i - 1] + previous_particles_[i].prob;
	
	newWeightU[0] = drand48();
	for (int i = 1; i < NUM_PARTICLES; i++)
		newWeightU[i] = newWeightU[i - 1] + 1.0;

	int i = 1;
	for (int j = 0; j < NUM_PARTICLES; j++) {

		while (newWeightU[j] > newWeightC[i])
			i = i + 1;
		particles_[j] = previous_particles_[i];
		particles_[j].prob = 1.0f;
	}

//	printParticles();
}

void LocalizationModule::copyParticles() {
	memcpy(localizationMem->particles, particles_,
			NUM_PARTICLES * sizeof(Particle));
}

void LocalizationModule::updateParticlesFromOdometry() {
//	std::cout << "Updating Particles From Odometry..." << std::endl;

	Pose2D disp = odometry->displacement;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.moveRelative(disp);
		p.degradeProbability(DEGRADE_FACTOR);
	}

//	randomWalkParticles();
}

void LocalizationModule::resetParticles() {
	std::cout << "Resetting Particles..." << std::endl;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.prob = 1.0f;
		p.placeRandomly();
	}

//	printParticles();
}

void LocalizationModule::setParticleProbabilities(float newProb) {
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.prob = newProb;
	}
}

void LocalizationModule::randomWalkParticles() {
	std::cout << "Performing Random Walk on All Particles..." << std::endl;
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

//		if (p1Ratio == 0.0)
//			p1Ratio = 0.05;
//		if (p2Ratio == 0.0)
//			p2Ratio = 0.05;

		float p1AngleRatio = p1Ratio;
		float p2AngleRatio = p2Ratio;

		part1.move(dPos * p1Ratio, p1AngleRatio * dAng);
		part2.move(-dPos * p2Ratio, p2AngleRatio * -dAng);
	}

//	printParticles();
}

void LocalizationModule::updatePose() {
	std::cout << "Updating Robot Pose..." << std::endl;
	WorldObject& self = worldObjects->objects_[robotState->WO_SELF];
	// Compute a weighted average of the particles to fill in your location

//	printParticles();

	float sumProb = 0.0;

	Point2D robotLoc(0.0, 0.0);
	AngRad robotOrient = 0.0;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		if (p.prob > 0.2) {
			robotLoc += p.loc * p.prob;
			robotOrient += p.theta * p.prob;
			sumProb += p.prob;
		}	
	}

	robotLoc /= sumProb;
	robotOrient /= sumProb;

	self.loc = robotLoc;
	self.orientation = robotOrient;

	std::cout << "Robot Location: " << robotLoc.x << ", " << robotLoc.y
			<< ", Robot Orientation: " << robotOrient << std::endl;
}

void LocalizationModule::printParticles() {
	std::cout << "------------------------------------------------"
			<< std::endl;
	std::cout << "Print Particle Information" << std::endl;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		std::cout << "[Particle " << i << "]: " << particles_[i].loc.x << ", "
				<< particles_[i].loc.y << ", " << particles_[i].theta << ", "
				<< particles_[i].prob << std::endl;
	}
	std::cout << "------------------------------------------------"
			<< std::endl;
}
