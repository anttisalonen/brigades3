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
};

class AIPlanner {
	public:
		AIPlanner() = default;
		AIPlanner(const WorldMap* wmap, const SoldierPhysics* phys);
		AITask getNextTask(const AISensor& sensor);
		void updateTask(const AISensor& sensor);

	private:
		const WorldMap* mMap;
		const SoldierPhysics* mPhys;
		AITask mCurrTask;
};

#endif

