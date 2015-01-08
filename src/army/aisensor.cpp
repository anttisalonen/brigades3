#include <common/Math.h>

#include "aisensor.hpp"

SoldierKnowledge::SoldierKnowledge(unsigned int id, const Common::Vector3& pos)
	: mLastKnownPosition(pos),
	mCurrentlySeen(true),
	mID(id)
{
}

AISensor::AISensor(const WorldMap* wmap, const Soldiers* soldiers, unsigned int id)
	: mMap(wmap),
	mSoldiers(soldiers),
	mPhys(soldiers->getPhys(id)),
	mID(id)
{
}

void AISensor::update(float dt)
{
	if(mTimer.check(dt)) {
		mSensedSoldiers.clear();
		auto& mypos = mPhys->getPosition();
		auto mydir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation());
		for(auto& s : mSoldiers->getSoldiersAt(mypos, 100.0f)) {
			if(s != mID && canSee(s) && mSoldiers->soldierIsAlive(s)) {
				auto& othpos = mSoldiers->getSoldierPosition(s);
				Common::Vector3 posdiff = othpos - mypos;
				auto ang = Common::Vector2(posdiff.x, posdiff.z).angleTo(Common::Vector2(mydir.x, mydir.z));
				if(ang < QUARTER_PI) {
					mSensedSoldiers.push_back(SoldierKnowledge(s, othpos));
				}
			}
		}
	}
}

std::vector<SoldierKnowledge> AISensor::getCurrentlySeenEnemies() const
{
	std::vector<SoldierKnowledge> ret;
	for(const auto& s : mSensedSoldiers) {
		if(s.isCurrentlySeen())
			ret.push_back(s);
	}
	return ret;
}

bool AISensor::canSee(unsigned int id) const
{
	// TODO: take trees into account
	auto sightpos = mPhys->getPosition() + Common::Vector3(0.0f, 1.7f, 0.0f);
	auto enemypos = mSoldiers->getSoldierPosition(id) + Common::Vector3(0.0f, 1.65f, 0.0f);
	return !mMap->lineBlockedByLand(sightpos,
			enemypos,
			nullptr) &&
		!mMap->lineBlockedByObstacles(sightpos, enemypos, false, nullptr);
}


