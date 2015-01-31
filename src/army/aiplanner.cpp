#include "random.hpp"
#include "aiplanner.hpp"

AIPlanner::AIPlanner(const WorldMap* wmap, const SoldierPhysics* phys)
	: mMap(wmap),
	mPhys(phys),
	mTimer(0.2f, Random::uniform(Random::SourceAI))
{
}

AITask AIPlanner::attackEnemy(std::vector<SoldierKnowledge>& enemies)
{
	auto mypos = mPhys->getPosition();
	std::sort(enemies.begin(), enemies.end(), [&](const SoldierKnowledge& a, const SoldierKnowledge& b) {
			return mypos.distance2(a.getPosition()) <
			mypos.distance2(b.getPosition()); });

	AITask t;

	t.Vec = enemies[0].getPosition();
	if(enemies[0].isCurrentlySeen() && mypos.distance(t.Vec) < 100.0f) {
		t.Type = AITask::Type::Shoot;
	} else {
		t.Type = AITask::Type::Move;
	}
	return t;
}

void AIPlanner::updateTask(const AISensor& sensor)
{
	// attack any seen enemies
	auto enemiesSeen = sensor.getCurrentlySeenEnemies();
	if(enemiesSeen.size() > 0) {
		mCurrTask = attackEnemy(enemiesSeen);
		return;
	}

	// if none seen but recollection exists, hunt
	auto enemyEntries = sensor.getEnemyEntries();
	if(enemyEntries.size() > 0) {
		mCurrTask = attackEnemy(enemyEntries);
		return;
	}

	if(mCurrTask.Type == AITask::Type::Idle) {
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

AITask AIPlanner::update(float dt, const AISensor& sensor)
{
	mCurrTask.ElapsedTime += dt;
	if(!mTimer.check(dt)) {
		updateTask(sensor);
	}

	return mCurrTask;
}


