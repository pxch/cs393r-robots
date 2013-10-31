#ifndef LOCALIZATION_MODULE_H
#define LOCALIZATION_MODULE_H

#include <Module.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/TeamPacketsBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/JointBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/ProcessedSonarBlock.h>
#include <memory/DelayedLocalizationBlock.h>
#include <localization/Particle.h>

#define RESAMPLE_FREQ 5
#define RANDOM_WALK_FREQ 5
#define DEGRADE_FACTOR 0.99
#define DELTA_DIST 20
#define DELTA_ANG (DEG_T_RAD * 45)

class LocalizationModule: public Module {
public:
	void specifyMemoryDependency();
	void specifyMemoryBlocks();
	void initSpecificModule();
	void processFrame();
private:
	void updatePose();
	void updateParticlesFromOdometry();
	void resetParticles();
	void setParticleProbabilities(float newProb);
	void randomWalkParticles();
	void copyParticles();

	void updateParticlesFromSensor();
	void updateParticlesFromBeacon(WorldObject* beacon);

	void resamplingParticles();
	void resamplingParticles2();
	int sampleIndexFromRandom(float random);

//	void computeParticleStats();

	void printParticles();

//	float probVarianceChange();

	bool trappedInWrongPosition;

	unsigned int innerFrameIndex;

	Particle particles_[NUM_PARTICLES];

	Particle previous_particles_[NUM_PARTICLES];

	float dist_bias_var;
	float dist_bias_mean;
	float ang_bias_var;
	float ang_bias_mean;

//	Point2D particle_loc_mean;
//	Point2D particle_loc_var;
//	Point2D particle_loc_var_prev;
//	AngRad particle_theta_mean;
//	AngRad particle_theta_var;
//	AngRad particle_theta_var_prev;
//
//	float particle_prob_mean;
//	float particle_prob_var;
//	float particle_prob_var_prev;

	WorldObjectBlock* worldObjects;
	LocalizationBlock* localizationMem;
	TeamPacketsBlock* teamPacketsMem;
	FrameInfoBlock* frameInfo;
	OdometryBlock* odometry;
	RobotStateBlock* robotState;
	GameStateBlock* gameState;
	JointBlock* jointAngles;
	BehaviorBlock* behaviorMem;
	ProcessedSonarBlock* processedSonar;
	DelayedLocalizationBlock* delayedLocalization;
};

#endif
