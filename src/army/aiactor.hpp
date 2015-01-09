#ifndef BRIGADES_AIACTOR_HPP
#define BRIGADES_AIACTOR_HPP

#include <common/Vector3.h>

#include "hittable.hpp"
#include "soldierphysics.hpp"
#include "aiplanner.hpp"

class AIActor {
	public:
		AIActor() = default;
		AIActor(SoldierPhysics* phys, ShooterComponent* shooter, float shootingSkill);
		void execute(const AITask& t);

	private:
		float turnTowards(const Common::Vector3& abspos, float variation);

		SoldierPhysics* mPhys;
		ShooterComponent* mShooter;
		float mVariation;
};

#endif

