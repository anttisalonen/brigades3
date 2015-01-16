#include "random.hpp"
#include "aiplanner.hpp"

AIPlanner::AIPlanner(const WorldMap* wmap, const SoldierPhysics* phys)
	: mMap(wmap),
	mPhys(phys)
{
}

void AIPlanner::updateTask(const AISensor& sensor)
{
	auto enemiesSeen = sensor.getCurrentlySeenEnemies();
	if(enemiesSeen.size() > 0) {
		auto mypos = mPhys->getPosition();
		std::sort(enemiesSeen.begin(), enemiesSeen.end(), [&](const SoldierKnowledge& a, const SoldierKnowledge& b) {
				return mypos.distance2(a.getPosition()) <
				mypos.distance2(b.getPosition()); });
		AITask t;
		t.Type = AITask::Type::Shoot;
		t.Vec = enemiesSeen[0].getPosition();
		mCurrTask = t;
	} else if(mCurrTask.Type == AITask::Type::Idle) {
		AITask t;
		t.Type = AITask::Type::Move;
		auto w2 = mMap->getWidth() * 0.5f;
		auto xtgt = Random::clamped(Random::SourceAI) * w2 + w2;
		auto ytgt = Random::clamped(Random::SourceAI) * w2 + w2;
		if(mMap->getHeightAt(xtgt, ytgt) > 0.0f) {
			t.Vec = Common::Vector3(xtgt, 0.0f, ytgt);
			mCurrTask = t;
		} // else idle one more step
	} else if(mCurrTask.Type == AITask::Type::Move) {
		if(mPhys->getPosition().distance(mCurrTask.Vec) < 10.0f) {
			mCurrTask = AITask();
		} else if(mCurrTask.ElapsedTime > 30.0f) {
			std::cout << "Timeout at " << mPhys->getPosition() << "\n";
			mCurrTask = AITask();
		}
		// else continue with existing task
	} else {
		mCurrTask = AITask();
	}
}

AITask AIPlanner::getNextTask(const AISensor& sensor)
{
	updateTask(sensor);
	return mCurrTask;
}

void AIPlanner::update(float dt)
{
	mCurrTask.ElapsedTime += dt;
}


