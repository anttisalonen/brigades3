#ifndef BRIGADES_AI_HPP
#define BRIGADES_AI_HPP

#include <vector>

#include "worldmap.hpp"
#include "hittable.hpp"
#include "soldiers.hpp"
#include "soldierphysics.hpp"
#include "aisensor.hpp"
#include "aiplanner.hpp"
#include "aiactor.hpp"

class AIComponent {
	public:
		AIComponent() = default;
		AIComponent(const WorldMap* wmap, Soldiers* soldiers, unsigned int id, float shootingSkill);
		void update(float dt);

	private:
		const Soldiers* mSoldiers;
		AISensor mSensor;
		AIPlanner mPlanner;
		AIActor mActor;
		SoldierPhysics* mPhys;
		ShooterComponent* mShooter;
		HittableComponent* mHittable;
};

struct AIConstants {
	float AvgShootingSkill;
	float MaxShootingSkillVariation;
};

class AI {
	public:
		AI() = default;
		AI(const WorldMap* wmap, Soldiers* soldiers, const AIConstants& aic);
		void init();
		void update(float dt);

	private:
		const WorldMap* mMap;
		Soldiers* mSoldiers;
		std::vector<AIComponent> mAIs;
		unsigned int mNumAIs;
		AIConstants mAIConstants;
};

#endif

