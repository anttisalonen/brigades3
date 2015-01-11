#include <common/Math.h>

#include "random.hpp"
#include "aiactor.hpp"

class Steering {
	public:
		Steering(const SoldierPhysics* phys);
		void arrive(const Common::Vector3& tgt, float radius);
		void turnTowards(float tgt);
		void turnTowards(const Common::Vector3& tgt);
		const Common::Vector3& getAcceleration() const;
		float getRotation() const;

	private:
		float accelLeft() const;
		void addAcceleration(const Common::Vector3& acc);

		const SoldierPhysics* mPhys;
		Common::Vector3 mAcceleration;
		float mRotation = 0.0f;
};

Steering::Steering(const SoldierPhysics* phys)
	: mPhys(phys)
{
}

void Steering::arrive(const Common::Vector3& tgt, float radius)
{
	auto diff = tgt - mPhys->getPosition();
	addAcceleration(diff);
}

void Steering::turnTowards(float tgtyaw)
{
	auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation());
	auto curryaw = atan2(currdir.z, currdir.x);
	auto diffyaw = curryaw - tgtyaw;
	mRotation += diffyaw;
}

void Steering::turnTowards(const Common::Vector3& tgt)
{
	auto tgtvec = tgt - mPhys->getPosition();
	auto tgtyaw = atan2(tgtvec.z, tgtvec.x);
	turnTowards(tgtyaw);
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

float Steering::accelLeft() const
{
	return SoldierPhysics::RunAcceleration - mAcceleration.length();
}

AIActor::AIActor(SoldierPhysics* phys, ShooterComponent* shooter, float shootingSkill)
	: mPhys(phys),
	mShooter(shooter)
{
	shootingSkill = Common::clamp(0.0f, shootingSkill, 1.0f);
	mVariation = (1.0f - shootingSkill) * 0.3f;
}

void AIActor::execute(const AITask& t)
{
	switch(t.Type) {
		case AITask::Type::Idle:
			break;

		case AITask::Type::Move:
			{
				Steering st(mPhys);
				st.arrive(t.Vec, 5.0f);
				st.turnTowards(t.Vec);
				auto accel = st.getAcceleration();
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
				// yaw
				auto tgtvec = t.Vec - mPhys->getPosition();
				auto tgtyaw = atan2(tgtvec.z, tgtvec.x);
				Steering st(mPhys);
				st.turnTowards(tgtyaw + Random::clamped(Random::SourceAI) * mVariation);
				auto diffyaw = st.getRotation();

				// pitch
				auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation() * mPhys->getAimPitch());
				auto currpitch = atan2(currdir.y, 1.0f);
				auto tgtpitch = atan2(tgtvec.normalized().y, 1.0f) + Random::clamped(Random::SourceAI) * mVariation;
				auto diffpitch = -(currpitch - tgtpitch);

				mPhys->addRotation(0.02f * diffyaw, 0.02f * diffpitch);
				if(fabs(diffyaw) < mVariation * 0.2f + 0.05f && fabs(diffpitch) < mVariation * 0.2f + 0.05f) {
					mShooter->shoot();
				}
			}
			break;
	}
}

