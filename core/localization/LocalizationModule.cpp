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

	innerFrameIndex = 1;
	resetParticles();
}

void LocalizationModule::processFrame() {
	int frameID = frameInfo->frame_id;
	std::cout << "Frame: " << innerFrameIndex << std::endl;

	// 1. Update particles from observations
	updateParticlesFromOdometry();
	updateParticlesFromSensor();

	// 2. If this is a resampling frame, resample
	if (innerFrameIndex % RESAMPLE_FREQ == 0)
		resamplingParticles2();

	// 3. Update the robot's pose
	updatePose();

	// 4. If this is a random walk frame, random walk
	if (innerFrameIndex % RANDOM_WALK_FREQ == 0)
		randomWalkParticles();

	// 5. Copy particles to localization memory:
	copyParticles();

	innerFrameIndex ++;
}

void LocalizationModule::updateParticlesFromSensor() {
	std::cout << "Updating Particles From Sensor..." << std::endl;

	// Reset Beacon Location in each Frame
	for (int i = 0; i < NUM_LANDMARKS; i++) {
		WorldObject *wo = &worldObjects->objects_[i + LANDMARK_OFFSET];
		wo->loc = landmarkLocation[i];

		// set heights
		if (wo->isGoal()) {
			wo->upperHeight = GOAL_HEIGHT;
			wo->lowerHeight = 0;
			wo->elevation = (wo->upperHeight + wo->lowerHeight) / 2;
		} else {
			wo->upperHeight = 0;
			wo->lowerHeight = 0;
			wo->elevation = 0;
		}
	}

	WorldObject* beacon_p_y = &worldObjects->objects_[WO_BEACON_PINK_YELLOW];
	WorldObject* beacon_y_p = &worldObjects->objects_[WO_BEACON_YELLOW_PINK];
	WorldObject* beacon_b_y = &worldObjects->objects_[WO_BEACON_BLUE_YELLOW];
	WorldObject* beacon_y_b = &worldObjects->objects_[WO_BEACON_YELLOW_BLUE];
	WorldObject* beacon_p_b = &worldObjects->objects_[WO_BEACON_PINK_BLUE];
	WorldObject* beacon_b_p = &worldObjects->objects_[WO_BEACON_BLUE_PINK];

	if (beacon_p_y->seen) {
//		std::cout << "------------------------------------------------"
//				<< std::endl;
//		std::cout << "Updating Particles from Beacon_Pink_Yellow" << std::endl;
		updateParticlesFromBeacon(beacon_p_y);
//		std::cout << "------------------------------------------------"
//				<< std::endl;
	}
	if (beacon_y_p->seen) {
//		std::cout << "------------------------------------------------"
//				<< std::endl;
//		std::cout << "Updating Particles from Beacon_Yellow_Pink" << std::endl;
		updateParticlesFromBeacon(beacon_y_p);
//		std::cout << "------------------------------------------------"
//				<< std::endl;
	}
	if (beacon_b_y->seen) {
//		std::cout << "------------------------------------------------"
//				<< std::endl;
//		std::cout << "Updating Particles from Beacon_Blue_Yellow" << std::endl;
		updateParticlesFromBeacon(beacon_b_y);
//		std::cout << "------------------------------------------------"
//				<< std::endl;
	}
	if (beacon_y_b->seen) {
//		std::cout << "------------------------------------------------"
//				<< std::endl;
//		std::cout << "Updating Particles from Beacon_Yellow_Blue" << std::endl;
		updateParticlesFromBeacon(beacon_y_b);
//		std::cout << "------------------------------------------------"
//				<< std::endl;
	}
	if (beacon_p_b->seen) {
//		std::cout << "------------------------------------------------"
//				<< std::endl;
//		std::cout << "Updating Particles from Beacon_Pink_Blue" << std::endl;
		updateParticlesFromBeacon(beacon_p_b);
//		std::cout << "------------------------------------------------"
//				<< std::endl;
	}
	if (beacon_b_p->seen) {
//		std::cout << "------------------------------------------------"
//				<< std::endl;
//		std::cout << "Updating Particles from Beacon_Blue_Pink" << std::endl;
		updateParticlesFromBeacon(beacon_b_p);
//		std::cout << "------------------------------------------------"
//				<< std::endl;
	}
}

void LocalizationModule::updateParticlesFromBeacon(WorldObject* beacon) {

//	float prob_seq[NUM_PARTICLES] = { 0.0 };
//	float prob_mean = 0.0;
//	for (int i = 0; i < NUM_PARTICLES; i++) {
//		prob_seq[i] = particles_[i].prob;
//		prob_mean += prob_seq[i];
//	}
//	prob_mean /= NUM_PARTICLES;
//	float prob_se = 0.0;
//	for (int i = 0; i < NUM_PARTICLES; i++) {
//		prob_se += (prob_seq[i] - prob_mean) * (prob_seq[i] - prob_mean);
//	}
//	prob_se /= NUM_PARTICLES;
//	prob_se = sqrt(prob_se);
//
//	std::cout << "Prob Stats: " << prob_mean << ", " << prob_se << std::endl;

	float normalDistance = beacon->visionDistance * beacon->visionDistance;
	float normalBearing = beacon->visionBearing * beacon->visionBearing;
	
//	float normalDistance = beacon->visionDistance * beacon->visionDistance;
//	float normalBearing = M_PI * M_PI;

	float distanceBias = 0.0;
	float bearingBias = 0.0;

	float particleDistance = 0.0;
	float particleBearing = 0.0;
      
        float minBias = 50000;
        float maxBias = 0.0;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		particleDistance = p.loc.getDistanceTo(beacon->loc);
		particleBearing = p.loc.getBearingTo(beacon->loc, p.theta);

		distanceBias = abs(beacon->visionDistance - particleDistance);
		bearingBias = abs(beacon->visionBearing - particleBearing);

                if(minBias > distanceBias) minBias = distanceBias;
		if(maxBias < distanceBias) maxBias = distanceBias;

		std::cout << "Beacon: " << beacon->loc.x << ", " << beacon->loc.y
				<< ". Particle[" << i << "]: " << p.loc.x << ", " << p.loc.y
				<< ", " << p.theta * 180 / M_PI << std::endl;
		std::cout << "Particle Parameter: " << particleDistance << ", "
				<< particleBearing * 180 / M_PI << std::endl;
		std::cout << "Beacon Parameter: " << beacon->visionDistance << ", "
				<< beacon->visionBearing * 180 / M_PI << std::endl;

		float prob_multiplier = exp(
				-0.05 * (distanceBias * distanceBias) / normalDistance
						- 0.05 * (bearingBias * bearingBias) / normalBearing);
		

		//float prob_nultiplier_distance = 1.0/sqrt(2*M_PI*normalDistance)*exp(distanceBias*distanceBias/(2*normalDistance));
			
		//float prob_nultiplier_angle = 1.0/sqrt(2*M_PI*normalBearing)*exp(distanceBias*distanceBias/(2*normalBearing));

	
		//float prob_multiplier = (prob_nultiplier_distance+prob_nultiplier_angle)/2;
		p.prob = p.prob * prob_multiplier;

		std::cout << "Prob Multiplier: " << prob_multiplier << std::endl;
	}


	if(abs(minBias - maxBias) < 500 and minBias > 500){

		resetParticles(); 
		return;
	} 


	float sumProb = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		sumProb += particles_[i].prob;
	}
	std::cout << "Prob Normalization: " << sumProb << std::endl;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		particles_[i].prob /= (sumProb / NUM_PARTICLES);
	}

}

void LocalizationModule::resamplingParticles() {
	std::cout << "------------------------------------------------"
			<< std::endl;
	std::cout << "Resampling Particles..." << std::endl;
	float sumProb = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		previous_particles_[i] = particles_[i];
		sumProb += particles_[i].prob;
	}
//	std::cout << "Prob Normalization: " << sumProb << std::endl;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		previous_particles_[i].prob /= sumProb;
	}

	int current_index = 0;
	int previous_index = -1;
	while (current_index < NUM_PARTICLES) {
		double random = drand48();
		previous_index = sampleIndexFromRandom(random);
		if (previous_index == -1)
			continue;
		else {
//			std::cout << "(" << random << ", " << previous_index << "), ";
			particles_[current_index] = previous_particles_[previous_index];
			particles_[current_index].prob = 1.0;
			current_index++;
		}
	}
//	std::cout << std::endl;
//	std::cout << "------------------------------------------------"
//			<< std::endl;
//
//	for (int i = 0; i < NUM_PARTICLES; i++) {
//		std::cout << "[Particle " << i << "]: " << particles_[i].loc.x << ", "
//				<< particles_[i].loc.y << std::endl;
//	}
	std::cout << "------------------------------------------------"
			<< std::endl;
}



void LocalizationModule::resamplingParticles2() {
		
	Particle newParticles[NUM_PARTICLES];
	int index=0;
	float newWeightC[NUM_PARTICLES];
	float newWeightU[NUM_PARTICLES];
	newWeightC[0] = particles_[0].prob;
	
	for(int i=1; i < NUM_PARTICLES; i++) newWeightC[i] = newWeightC[i-1] + particles_[i].prob;
	newWeightU[0] = 1/NUM_PARTICLES;
	int i = 1;

	for(int j = 0; j < NUM_PARTICLES; j++){

		while(newWeightU[j] > newWeightC[i]) i = i+1;
		Particle& p = particles_[i];
		p.prob = 1/NUM_PARTICLES;
		newParticles[index] = p;
		index = index + 1;
		newWeightU[j+1] = newWeightU[j] + 1/NUM_PARTICLES;
	} 

	for(int j = 0; j < NUM_PARTICLES; j++){

		particles_[j] = newParticles[j];
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
	std::cout << "Updating Particles From Odometry..." << std::endl;

	Pose2D disp = odometry->displacement;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.moveRelative(disp);
		p.degradeProbability(DEGRADE_FACTOR);
	}
}

void LocalizationModule::resetParticles() {
	std::cout << "Resetting Particles..." << std::endl;

	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.prob = 1.0f;
		p.placeRandomly();
	}

//	for (int i = 0; i < NUM_PARTICLES; i++) {
//		std::cout << "[Particle " << i << ": " << particles_[i].loc.x << ", "
//				<< particles_[i].loc.y << "], ";
//	}
//
//	std::cout << std::endl << "Particles been reset..." << std::endl;
}

void LocalizationModule::setParticleProbabilities(float newProb) {
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		p.prob = newProb;
	}
}

void LocalizationModule::randomWalkParticles() {
	std::cout << "------------------------------------------------"
			<< std::endl;
	std::cout << "Performing Random Walk on All Particles..." << std::endl;
	// loop through half, moving each pair of particles opposite directions

	for (int i = 0; i < NUM_PARTICLES / 2; i++) {
		Particle& part1 = particles_[i];
		Particle& part2 = particles_[i + NUM_PARTICLES / 2];

		Vector2D dPos(DELTA_DIST * (2.0 * drand48() - 1),
				DELTA_DIST * (2.0 * drand48() - 1));
		AngRad dAng = DELTA_ANG * (2.0 * drand48() - 1);

//		std::cout << "Particle Index: [" << i << ", " << i + NUM_PARTICLES / 2
//				<< "]: dPos = (" << dPos.x << ", " << dPos.y << "), dAng = "
//				<< dAng * 180 / M_PI << std::endl;

		// move them in opposite directions on this vector, based on their prob
		float p1Ratio = 1.0 - part1.prob;
		float p2Ratio = 1.0 - part2.prob;
		
		if (p1Ratio == 0.0)
			p1Ratio = 0.05;
		if (p2Ratio == 0.0)
			p2Ratio = 0.05;

//		float p1Ratio = 1;
//		float p2Ratio = 1;

		float p1AngleRatio = p1Ratio;
		float p2AngleRatio = p2Ratio;

		part1.move(dPos * p1Ratio, p1AngleRatio * dAng);
		part2.move(-dPos * p2Ratio, p2AngleRatio * -dAng);

	}
//	std::cout << "------------------------------------------------"
//			<< std::endl;
//
//	for (int i = 0; i < NUM_PARTICLES; i++) {
//		std::cout << "[Particle " << i << "]: " << particles_[i].loc.x << ", "
//				<< particles_[i].loc.y << std::endl;
//	}

	std::cout << "------------------------------------------------"
			<< std::endl;
}

void LocalizationModule::updatePose() {
	std::cout << "------------------------------------------------"
			<< std::endl;
	std::cout << "Updating Robot Pose..." << std::endl;
	WorldObject& self = worldObjects->objects_[robotState->WO_SELF];
	// Compute a weighted average of the particles to fill in your location

	Point2D robotLoc(0.0, 0.0);
	AngRad robotOrient = 0.0;

	float sumProb = 0.0;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		sumProb += particles_[i].prob;
	}
	std::cout << "Prob Normalization: " << sumProb << std::endl;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		particles_[i].prob /= (sumProb / NUM_PARTICLES);
	}

//	int effectiveParticlesCount = 0;
//	float probThres = 0.2;
	for (int i = 0; i < NUM_PARTICLES; i++) {
		Particle& p = particles_[i];
		robotLoc += p.loc * p.prob;
		robotOrient += p.theta * p.prob;
//		effectiveParticlesCount ++;
	}

	robotLoc /= NUM_PARTICLES;
	robotOrient /= NUM_PARTICLES;

	self.loc = robotLoc;
	self.orientation = robotOrient;

	std::cout << "Robot Location: " << robotLoc.x << ", " << robotLoc.y
			<< ", Robot Orientation: " << robotOrient << std::endl;
	std::cout << "------------------------------------------------"
			<< std::endl;
}
