#include "random.hpp"
#include "ai.hpp"

AIComponent::AIComponent(const WorldMap* wmap, Soldiers* soldiers, unsigned int id, float shootingSkill)
	: mSoldiers(soldiers),
	mSensor(wmap, soldiers, id),
	mPlanner(wmap, soldiers->getPhys(id)),
	mActor(soldiers->getPhys(id), soldiers->getShooter(id), shootingSkill),
	mPhys(soldiers->getPhys(id)),
	mShooter(soldiers->getShooter(id)),
	mHittable(soldiers->getHittable(id))
{
}

void AIComponent::update(float dt)
{
	if(mHittable->hasDied())
		return;

	mSensor.update(dt);
	auto task = mPlanner.getNextTask(mSensor);
	mActor.execute(task);
}

AI::AI(const WorldMap* wmap, Soldiers* soldiers, const AIConstants& aic)
	: mMap(wmap),
	mSoldiers(soldiers),
	mAIConstants(aic)
{
	mAIs.resize(Soldiers::MAX_SOLDIERS);
}

void AI::init()
{
	mNumAIs = mSoldiers->getNumSoldiers();
	for(unsigned int i = 0; i < mNumAIs; i++) {
		auto shootingSkill = mAIConstants.AvgShootingSkill +
			Random::clamped(Random::SourceAI) * mAIConstants.MaxShootingSkillVariation;
		mAIs[i] = AIComponent(mMap, mSoldiers, i, shootingSkill);
	}
}

void AI::update(float dt)
{
	auto plid = mSoldiers->getPlayerSoldierIndex();
	for(unsigned int i = 0; i < mNumAIs; i++) {
		if(i != plid) {
			mAIs[i].update(dt);
		}
	}
}


