#ifndef BRIGADES_AISENSOR_HPP
#define BRIGADES_AISENSOR_HPP

#include <vector>

#include <common/Vector3.h>
#include <common/Clock.h>

#include "worldmap.hpp"
#include "soldierphysics.hpp"
#include "soldiers.hpp"

class SoldierKnowledge {
	public:
		SoldierKnowledge(unsigned int id, const Common::Vector3& pos);
		bool isCurrentlySeen() const { return mCurrentlySeen; }
		const Common::Vector3& getPosition() const { return mLastKnownPosition; }
		unsigned int getID() const { return mID; }

	private:
		Common::Vector3 mLastKnownPosition;
		bool mCurrentlySeen;
		unsigned int mID;
};

class AISensor {
	public:
		AISensor();
		AISensor(const WorldMap* wmap, const Soldiers* soldiers, unsigned int id);
		void update(float dt);
		std::vector<SoldierKnowledge> getCurrentlySeenEnemies() const;
		bool canSee(unsigned int id) const;

	private:
		const WorldMap* mMap;
		const Soldiers* mSoldiers;
		const SoldierPhysics* mPhys;
		std::vector<SoldierKnowledge> mSensedSoldiers;
		unsigned int mID;
		Common::SteadyTimer mTimer = Common::SteadyTimer(0.2f);
};

#endif

