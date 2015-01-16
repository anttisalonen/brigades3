#include "random.hpp"
#include "ai.hpp"
#include "aistatic.hpp"

AIComponent::AIComponent(const WorldMap* wmap, Soldiers* soldiers, unsigned int id, float shootingSkill)
	: mSoldiers(soldiers),
	mSensor(wmap, soldiers, id),
	mPlanner(wmap, soldiers->getPhys(id)),
	mActor(wmap, soldiers->getPhys(id), soldiers->getShooter(id), shootingSkill, id),
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
	mPlanner.update(dt);
	auto task = mPlanner.getNextTask(mSensor);
	mActor.execute(task);
}

AI::AI(const WorldMap* wmap, Soldiers* soldiers, const AIConstants& aic, Scene::Scene* scene)
	: mMap(wmap),
	mSoldiers(soldiers),
	mAIConstants(aic),
	mScene(scene)
{
	if(aic.AIDebug)
		AIStatic::Debug::enable();
	mAIs.resize(Soldiers::MAX_SOLDIERS);
}

void AI::init()
{
	AIStatic::setWorldMap(mMap);
	AIStatic::Debug::setScene(mScene);
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


