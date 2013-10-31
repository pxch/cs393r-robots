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

	particle_loc_mean = Point2D(0.0, 0.0);
	particle_loc_var = Point2D(0.0, 0.0);
	particle_loc_var_prev = Point2D(0.0, 0.0);

	particle_theta_mean = 0.0;
	particle_theta_var = 0.0;
	particle_theta_var_prev = 0.0;
}

void LocalizationModule::computeParticleStats() {
	particle_loc_var_prev = particle_loc_var;
	particle_theta_var_prev = particle_theta_var;

	float mean_loc_x = 0, mean_loc_y = 0, mean_theta = 0;
	float var_loc_x = 0, var_loc_y = 0, var_theta = 0;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		mean_loc_x += particles_[i].loc.x;
		mean_loc_y += particles_[i].loc.y;
		mean_theta += particles_[i].theta;
	}
	mean_loc_x /= NUM_PARTICLES;
	mean_loc_y /= NUM_PARTICLES;
	mean_theta /= NUM_PARTICLES;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		var_loc_x += (particles_[i].loc.x - mean_loc_x)
				* (particles_[i].loc.x - mean_loc_x);
		var_loc_y += (particles_[i].loc.y - mean_loc_y)
				* (particles_[i].loc.y - mean_loc_y);
		var_theta += (particles_[i].theta - mean_theta)
				* (particles_[i].theta - mean_theta);
	}
	var_loc_x = sqrt(var_loc_x / NUM_PARTICLES);
	var_loc_y = sqrt(var_loc_y / NUM_PARTICLES);
	var_theta = sqrt(var_theta / NUM_PARTICLES);

	particle_loc_mean.x = mean_loc_x;
	particle_loc_mean.y = mean_loc_y;
	particle_theta_mean = mean_theta;

	particle_loc_var.x = var_loc_x;
	particle_loc_var.y = var_loc_y;
	particle_theta_var = var_theta;

	std::cout << "Particle Stats:" << std::endl;
	std::cout << particle_loc_mean.x << ", " << particle_loc_mean.y << ", " << particle_theta_mean << std::endl;
	std::cout << particle_loc_var.x << ", " << particle_loc_var.y << ", " << particle_theta_var << std::endl;
}

void LocalizationModule::processFrame() {
	int frameID = frameInfo->frame_id;
	std::cout << "Frame: " << innerFrameIndex << std::endl;

	computeParticleStats();

//	if (dist_bias_mean > 500 && dist_bias_var < 2000)
//		randomWalkParticles(200, M_PI / 4);

	// 1. Update particles from observations
	updateParticlesFromOdometry();
	updateParticlesFromSensor();

	// 2. If this is a resampling frame, resample
	if (innerFrameIndex % RESAMPLE_FREQ == 0)
		resamplingParticles();

	// 3. Update the robot's pose
	updatePose();

	// 4. If this is a random walk frame, random walk
	if (innerFrameIndex % RANDOM_WALK_FREQ == 0)
		randomWalkParticles(20, M_PI / 12);

	// 5. Copy particles to localization memory:
	copyParticles();

	innerFrameIndex++;
}

void LocalizationModule::updateParticlesFromSensor() {
	std::cout << "Updating Particles From Sensor..." << std::endl;

	// Reset Beacon Location in each Frame
	for (int i = NUM_LANDMARKS - 6; i < NUM_LANDMARKS; i++) {
		WorldObject *beacon = &worldObjects->objects_[i + LANDMARK_OFFSET];
		beacon->loc = landmarkLocation[i];
		if (beacon->seen)
			updateParticlesFromBeacon(beacon);
	}
}

void LocalizationModule::updateParticlesFromBeacon(WorldObject* beacon) {

	float normalDistance = beacon->visionDistance * beacon->visionDistance;
	float normalBearing = beacon->visionBearing * beacon->visionBearing;

	float distanceBias[NUM_PARTICLES];
	float bearingBias[NUM_PARTICLES];

	float particleDistance = 0.0;
	float particleBearing = 0.0;

	float minBias = 200000;
	float maxBias = 0.0;

	dist_bias_var = 0.0;
	dist_bias_mean = 0.0;
	ang_bias_var = 0.0;
	ang_bias_mean = 0.0;

	float temp;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		particleDistance = p.loc.getDistanceTo(beacon->loc);
		particleBearing = p.loc.getBearingTo(beacon->loc, p.theta);

//		std::cout << "Beacon: " << beacon->loc.x << ", " << beacon->loc.y
//				<< ". Particle[" << i << "]: " << p.loc.x << ", " << p.loc.y
//				<< ", " << p.theta * 180 / M_PI << std::endl;
//		std::cout << "Particle Parameter: " << particleDistance << ", "
//				<< particleBearing * 180 / M_PI << std::endl;
//		std::cout << "Beacon Parameter: " << beacon->visionDistance << ", "
//				<< beacon->visionBearing * 180 / M_PI << std::endl;

		distanceBias[i] = abs(beacon->visionDistance - particleDistance);
		bearingBias[i] = abs(beacon->visionBearing - particleBearing);

		if (minBias > distanceBias[i])
			minBias = distanceBias[i];
		if (maxBias < distanceBias[i])
			maxBias = distanceBias[i];

		dist_bias_mean += distanceBias[i];
		ang_bias_mean += bearingBias[i];
	}

	dist_bias_mean /= NUM_PARTICLES;
	ang_bias_mean /= NUM_PARTICLES;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		dist_bias_var += (distanceBias[i] - dist_bias_mean)
				* (distanceBias[i] - dist_bias_mean);
		ang_bias_var += (bearingBias[i] - ang_bias_mean)
				* (bearingBias[i] - ang_bias_mean);
	}

	dist_bias_var /= NUM_PARTICLES;
	if (dist_bias_var <= 0)
		return;
	ang_bias_var /= NUM_PARTICLES;
	if (ang_bias_var <= 0)
		return;

	std::cout << "Bias Stats: " << dist_bias_mean << ", " << dist_bias_var
			<< ", " << ang_bias_mean << ", " << ang_bias_var << std::endl;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];

		float prob_degrade = exp(
				-0.5 * (distanceBias[i] * distanceBias[i]) / normalDistance
						- 0.5 * (bearingBias[i] * bearingBias[i])
								/ normalBearing);

		p.degradeProbability(prob_degrade);

//		std::cout << "Prob Degrade Factor for Particle [" << i << "]: " << prob_degrade << std::endl;
	}

//	if (abs(minBias - maxBias) < 500 and minBias > 500) {
//		resetParticles();
//		return;
//	}

	//Normalize probability mean value equals one
	float sumProb = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		sumProb += particles_[i].prob;
	}
	std::cout << "Prob Normalization Factor: " << sumProb << std::endl;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		particles_[i].prob /= (sumProb / NUM_PARTICLES);
	}

	if (isnan(sumProb)) {
		std::cout << "ERROR!" << std::endl;
		printParticles();
		std::cin >> temp;
	}
}

void LocalizationModule::resamplingParticles() {
	std::cout << "Resampling Particles..." << std::endl;

	int current_index = 0;
	int previous_index = -1;
	while (current_index < NUM_PARTICLES) {
		double random = drand48() * NUM_PARTICLES;
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

//	printParticles();
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

void LocalizationModule::resamplingParticles2() {
	Particle newParticles[NUM_PARTICLES];
	int index = 0;
	float newWeightC[NUM_PARTICLES];
	float newWeightU[NUM_PARTICLES];
	newWeightC[0] = particles_[0].prob;

	for (int i = 1; i < NUM_PARTICLES; i++)
		newWeightC[i] = newWeightC[i - 1] + particles_[i].prob;
	newWeightU[0] = drand48();
	int i = 1;

	for (int j = 0; j < NUM_PARTICLES; j++) {

		while (newWeightU[j] > newWeightC[i])
			i = i + 1;
		Particle& p = particles_[i];
		p.prob = 1.0;
		newParticles[index] = p;
		index = index + 1;
		newWeightU[j + 1] = newWeightU[j] + 1.0;
	}

	for (int j = 0; j < NUM_PARTICLES; j++) {
		particles_[j] = newParticles[j];
	}

//	printParticles();
}

void LocalizationModule::copyParticles() {
	memcpy(localizationMem->particles, particles_,
			NUM_PARTICLES * sizeof(Particle));
}

void LocalizationModule::updateParticlesFromOdometry() {
	std::cout << "Updating Particles From Odometry..." << std::endl;

	Pose2D disp = odometry->displacement;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.moveRelative(disp);
		p.degradeProbability(DEGRADE_FACTOR);
	}

	randomWalkParticles(20, M_PI / 12);
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

void LocalizationModule::randomWalkParticles(float delta_dist,
		float delta_ang) {
	std::cout << "Performing Random Walk on All Particles..." << std::endl;
	// loop through half, moving each pair of particles opposite directions

	for (int i = 0; i < NUM_PARTICLES / 2; i++) {
		Particle& part1 = particles_[i];
		Particle& part2 = particles_[i + NUM_PARTICLES / 2];

		Vector2D dPos(DELTA_DIST * (2.0 * drand48() - 1),
				delta_dist * (2.0 * drand48() - 1));
		AngRad dAng = delta_ang * (2.0 * drand48() - 1);

// move them in opposite directions on this vector, based on their prob
		float p1Ratio = 1.0 - part1.prob;
		float p2Ratio = 1.0 - part2.prob;

		if (p1Ratio == 0.0)
			p1Ratio = 0.05;
		if (p2Ratio == 0.0)
			p2Ratio = 0.05;

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

	float sumProb = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		sumProb += particles_[i].prob;
	}
	std::cout << "Prob Normalization Factor: " << sumProb << std::endl;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		particles_[i].prob /= (sumProb / NUM_PARTICLES);
	}

	printParticles();

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

void LocalizationModule::printParticles() {
	std::cout << "------------------------------------------------"
			<< std::endl;
	std::cout << "Print Particle Information" << std::endl;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		std::cout << "[Particle " << i << "]: " << particles_[i].loc.x << ", "
				<< particles_[i].loc.y << std::endl;
	}
	std::cout << "------------------------------------------------"
			<< std::endl;
}
