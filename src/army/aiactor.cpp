#include <float.h>

#include <common/Math.h>

#include "random.hpp"
#include "aiactor.hpp"
#include "aistatic.hpp"

class Steering {
	public:
		Steering(const WorldMap* wmap, const SoldierPhysics* phys);
		void seek(const Common::Vector3& tgt);
		void avoidWalls();
		void avoidTrees();
		void turnTowards(float tgt);
		void turnTowards(const Common::Vector3& tgt);
		void lookWhereYoureGoing();
		const Common::Vector3& getAcceleration() const;
		float getRotation() const;

	private:
		float accelLeft() const;
		void addAcceleration(const Common::Vector3& acc);
		void maximiseAcceleration();

		const SoldierPhysics* mPhys;
		const WorldMap* mMap;
		Common::Vector3 mAcceleration;
		float mRotation = 0.0f;
};

Steering::Steering(const WorldMap* wmap, const SoldierPhysics* phys)
	: mPhys(phys),
	mMap(wmap)
{
}

void Steering::seek(const Common::Vector3& tgt)
{
	auto diff = tgt - mPhys->getPosition();
	addAcceleration(diff.normalized() * SoldierPhysics::RunAcceleration);
}

void Steering::avoidWalls()
{
	static const float LookAhead = 5.0f; // meters

	auto rayVec = Common::Vector2(mPhys->getVelocity().x, mPhys->getVelocity().z);
	if(rayVec.null())
		return;

	rayVec.normalize();
	rayVec *= LookAhead;

	Common::Vector2 feelers[] = {
		rayVec,
		Common::Math::rotate2D(rayVec, QUARTER_PI),
		Common::Math::rotate2D(rayVec, -QUARTER_PI),
	};

	const auto& pos = mPhys->getPosition();

	bool added = false;
	for(auto& f : feelers) {
		auto f3 = mMap->pointToVec(f.x, f.y) + Common::Vector3(0.0f, 1.0f, 0.0f);
		auto coll = AIStatic::getWallCollision(pos, pos + f3);

		if(!coll.Found)
			continue;

		auto thisdist = coll.Position.distance(pos);
		auto overshoot = LookAhead - thisdist;
		auto acc = Common::Vector3(coll.Normal.x, 0.0f, coll.Normal.y) * overshoot * overshoot;
		mAcceleration += acc;
		added = true;
	}
	if(added)
		maximiseAcceleration();
}

void Steering::avoidTrees()
{
	auto myvel = Common::Vector2(mPhys->getVelocity().x, mPhys->getVelocity().z);
	auto speed = myvel.length();
	if(speed < 0.02f)
		return;

	auto mypos = Common::Vector2(mPhys->getPosition().x, mPhys->getPosition().z);
	auto trees = mMap->getTreesAt(mypos.x, mypos.y, 5.0f + speed);

	bool added = false;
	for(const auto& tree : trees) {
		auto relpos = Common::Vector2(tree.Position.x - mypos.x, tree.Position.z - mypos.y);
		auto dist = relpos.length();
		if(dist > tree.Radius + 5.0f)
			continue;

		mAcceleration += Common::Vector3(-relpos.x, 0.0f, -relpos.y).normalized() * (tree.Radius + 5.0f - dist);
		added = true;
	}

	if(added)
		maximiseAcceleration();
}

void Steering::turnTowards(float tgtyaw)
{
	auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation());
	auto curryaw = atan2(currdir.z, currdir.x);
	auto diffyaw = curryaw - tgtyaw;
	if(diffyaw < -PI)
		diffyaw += TWO_PI;
	if(diffyaw > PI)
		diffyaw -= TWO_PI;
	mRotation += diffyaw;
}

void Steering::turnTowards(const Common::Vector3& tgt)
{
	auto tgtvec = tgt - mPhys->getPosition();
	auto tgtyaw = atan2(tgtvec.z, tgtvec.x);
	turnTowards(tgtyaw);
}

void Steering::lookWhereYoureGoing()
{
	turnTowards(mPhys->getPosition() + mAcceleration);
}

const Common::Vector3& Steering::getAcceleration() const
{
	return mAcceleration;
}

float Steering::getRotation() const
{
	return mRotation;
}

void Steering::addAcceleration(const Common::Vector3& acc)
{
	auto acc2 = acc;
	acc2.y = 0.0f;
	acc2.truncate(accelLeft());
	mAcceleration += acc2;
}

void Steering::maximiseAcceleration()
{
	mAcceleration.normalize();
	mAcceleration *= SoldierPhysics::RunAcceleration;
}

float Steering::accelLeft() const
{
	return SoldierPhysics::RunAcceleration - mAcceleration.length();
}

AIActor::AIActor(const WorldMap* wmap, SoldierPhysics* phys, ShooterComponent* shooter, float shootingSkill, unsigned int id)
	: mMap(wmap),
	mPhys(phys),
	mShooter(shooter),
	mID(id)
{
	shootingSkill = Common::clamp(0.0f, shootingSkill, 1.0f);
	mVariation = (1.0f - shootingSkill) * 0.3f;
}

void AIActor::execute(const AITask& t)
{
	switch(t.Type) {
		case AITask::Type::Idle:
			mPhys->setAiming(false);
			break;

		case AITask::Type::Move:
			{
				mPhys->setAiming(false);
				Steering st(mMap, mPhys);
				st.seek(t.Vec);
				st.avoidWalls();
				st.avoidTrees();
				st.lookWhereYoureGoing();
				auto accel = st.getAcceleration();
				AIStatic::Debug::drawLine(mID, "Moving",
						mPhys->getPosition() + Common::Vector3(0.0f, 1.5f, 0.0f),
						mPhys->getPosition() + accel + Common::Vector3(0.0f, 1.5f, 0.0f),
						Common::Color::Blue);
				if(accel.length() > 0.1f) {
					mPhys->addMovementAcceleration(accel);
				}
				auto rot = st.getRotation();
				if(fabs(rot) > 0.01f) {
					mPhys->addRotation(rot, 0.0f);
				}
			}
			break;

		case AITask::Type::Shoot:
			{
				mPhys->setAiming(true);
				// yaw
				auto tgtvec = t.Vec - mPhys->getPosition();
				auto tgtyaw = atan2(tgtvec.z, tgtvec.x);
				Steering st(mMap, mPhys);
				st.turnTowards(tgtyaw + Random::clamped(Random::SourceAI) * mVariation);
				auto diffyaw = st.getRotation();

				// pitch
				auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation() * mPhys->getAimPitch());
				auto currpitch = atan2(currdir.y, 1.0f);
				auto tgtpitch = atan2(tgtvec.normalized().y, 1.0f) + Random::clamped(Random::SourceAI) * mVariation;
				auto diffpitch = -(currpitch - tgtpitch);

				mPhys->addRotation(0.2f * diffyaw, 0.2f * diffpitch);
				if(fabs(diffyaw) < mVariation * 0.2f + 0.05f && fabs(diffpitch) < mVariation * 0.2f + 0.05f) {
					mShooter->shoot();
				}
			}
			break;
	}
}

