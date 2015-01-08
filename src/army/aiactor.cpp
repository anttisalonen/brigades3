#include <common/Math.h>
#include <common/Random.h>

#include "aiactor.hpp"

AIActor::AIActor(SoldierPhysics* phys, ShooterComponent* shooter)
	: mPhys(phys),
	mShooter(shooter)
{
}

void AIActor::execute(const AITask& t)
{
	switch(t.Type) {
		case AITask::Type::Idle:
			break;

		case AITask::Type::Move:
			{
				Common::Vector3 accel;
				accel = t.Vec - mPhys->getPosition();
				accel.y = 0.0f;
				if(accel.length() > 1.0f) {
					mPhys->addAcceleration(accel.normalized() * SoldierPhysics::RunAcceleration);
					turnTowards(t.Vec);
				}
			}
			break;

		case AITask::Type::Shoot:
			{
				auto diffyaw = turnTowards(t.Vec);
				auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation() * mPhys->getAimPitch());
				auto tgtvec = t.Vec - mPhys->getPosition();
				auto currpitch = atan2(currdir.y, 1.0f);
				auto tgtpitch = atan2(tgtvec.normalized().y, 1.0f) + Common::Random::clamped() * 0.2f;
				auto diffpitch = -(currpitch - tgtpitch);
				mPhys->rotate(0.0f, 0.02f * diffpitch);
				if(fabs(diffyaw) < 0.2f && fabs(diffpitch) < 0.2f) {
					mShooter->shoot();
				}
			}
			break;
	}
}

float AIActor::turnTowards(const Common::Vector3& abspos)
{
	auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation());
	auto tgtvec = abspos - mPhys->getPosition();
	auto curryaw = atan2(currdir.z, currdir.x);
	auto tgtyaw = atan2(tgtvec.z, tgtvec.x) + Common::Random::clamped() * 0.2f;
	auto diffyaw = curryaw - tgtyaw;
	mPhys->rotate(0.02f * diffyaw, 0.0f);
	return diffyaw;
}


