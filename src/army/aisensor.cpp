#include <common/Math.h>

#include "aisensor.hpp"
#include "random.hpp"

SoldierKnowledge::SoldierKnowledge(unsigned int id, const Common::Vector3& knownpos, bool currentlyseen, float currtime)
	: mLastKnownPosition(knownpos),
	mCurrentlySeen(currentlyseen),
	mID(id),
	mEntryTime(currtime)
{
}

AISensor::AISensor()
	: mTimer(0.2f)
{
}

AISensor::AISensor(const WorldMap* wmap, const Soldiers* soldiers, unsigned int id)
	: mMap(wmap),
	mSoldiers(soldiers),
	mPhys(soldiers->getPhys(id)),
	mID(id),
	mTimer(0.2f, Random::uniform(Random::SourceAI))
{
}

bool AISensor::canSeeEnemy(unsigned int id)
{
	if(id == mID)
		return false;

	if(!LOSNotBlocked(id))
		return false;

	const auto& othpos = mSoldiers->getSoldierPosition(id);
	const auto& mypos = mPhys->getPosition();
	Common::Vector3 posdiff = othpos - mypos;
	auto mydir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation());
	auto ang = Common::Vector2(posdiff.x, posdiff.z).angleTo(Common::Vector2(mydir.x, mydir.z));

	return ang < QUARTER_PI;
}

void AISensor::update(float dt)
{
	mCurrTime += dt;

	if(!mTimer.check(dt))
		return;

	auto oldSensed = mSensedSoldiers;
	mSensedSoldiers.clear();

	static const float MemorySpan = 10.0f; // seconds
	static const float EnemyLostRange = 5.0f; // forget enemy if he's not seen
	for(const auto& s : oldSensed) {
		if(!canSeeEnemy(s.first)) {
			if(s.second.timeSinceEntry(mCurrTime) < MemorySpan &&
					s.second.getPosition().distance(mPhys->getPosition()) > EnemyLostRange) {
				mSensedSoldiers.insert({s.first,
						SoldierKnowledge(s.second.getID(), s.second.getPosition(), false, s.second.entryTime())});
			}
		}
	}

	static const float VisibilityRange = 200.0f;
	for(auto& s : mSoldiers->getSoldiersAt(mPhys->getPosition(), VisibilityRange)) {
		if(canSeeEnemy(s)) {
			if(mSoldiers->soldierIsAlive(s)) {
				mSensedSoldiers.insert({s, SoldierKnowledge(s, mSoldiers->getSoldierPosition(s), true, mCurrTime)});
			} else {
				mSensedSoldiers.erase(s);
			}
		}
	}
}

std::vector<SoldierKnowledge> AISensor::getCurrentlySeenEnemies() const
{
	std::vector<SoldierKnowledge> ret;
	for(const auto& s : mSensedSoldiers) {
		if(s.second.isCurrentlySeen())
			ret.push_back(s.second);
	}
	return ret;
}

std::vector<SoldierKnowledge> AISensor::getEnemyEntries() const
{
	std::vector<SoldierKnowledge> ret;
	for(const auto& s : mSensedSoldiers) {
		ret.push_back(s.second);
	}
	return ret;
}

bool AISensor::LOSNotBlocked(unsigned int id) const
{
	// TODO: take trees into account
	auto sightpos = mPhys->getPosition() + Common::Vector3(0.0f, 1.7f, 0.0f);
	auto enemypos = mSoldiers->getSoldierPosition(id) + Common::Vector3(0.0f, 1.65f, 0.0f);
	return !mMap->lineBlockedByLand(sightpos,
			enemypos,
			nullptr) &&
		!mMap->lineBlockedByObstacles(sightpos, enemypos, false, nullptr);
}


