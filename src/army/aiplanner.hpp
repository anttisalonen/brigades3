#ifndef BRIGADES_AIPLANNER_HPP
#define BRIGADES_AIPLANNER_HPP

#include <common/Vector3.h>

#include "worldmap.hpp"
#include "soldierphysics.hpp"
#include "aisensor.hpp"

class AITask {
	public:
		enum class Type {
			Idle,
			Move,
			Shoot
		};

		Common::Vector3 Vec;
		Type Type = Type::Idle;
		float ElapsedTime = 0.0f;
};

class AIPlanner {
	public:
		AIPlanner() = default;
		AIPlanner(const WorldMap* wmap, const SoldierPhysics* phys);
		AITask update(float dt, const AISensor& sensor);

	private:
		void updateTask(const AISensor& sensor);

		const WorldMap* mMap;
		const SoldierPhysics* mPhys;
		AITask mCurrTask;
		Common::SteadyTimer mTimer = Common::SteadyTimer(0.2f);
};

#endif

