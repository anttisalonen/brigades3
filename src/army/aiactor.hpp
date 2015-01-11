#ifndef BRIGADES_AIACTOR_HPP
#define BRIGADES_AIACTOR_HPP

#include <common/Vector3.h>

#include "hittable.hpp"
#include "soldierphysics.hpp"
#include "aiplanner.hpp"

class AIActor {
	public:
		AIActor() = default;
		AIActor(const WorldMap* wmap, SoldierPhysics* phys, ShooterComponent* shooter, float shootingSkill, unsigned int id);
		void execute(const AITask& t);

	private:
		const WorldMap* mMap;
		SoldierPhysics* mPhys;
		ShooterComponent* mShooter;
		float mVariation;
		unsigned int mID;
};

#endif

