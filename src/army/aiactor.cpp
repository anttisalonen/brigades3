#include <common/Math.h>

#include "random.hpp"
#include "aiactor.hpp"

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
				Common::Vector3 accel;
				accel = t.Vec - mPhys->getPosition();
				accel.y = 0.0f;
				if(accel.length() > 1.0f) {
					mPhys->addAcceleration(accel.normalized() * SoldierPhysics::RunAcceleration);
					turnTowards(t.Vec, 0.0f);
				}
			}
			break;

		case AITask::Type::Shoot:
			{
				auto diffyaw = turnTowards(t.Vec, mVariation);
				auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation() * mPhys->getAimPitch());
				auto tgtvec = t.Vec - mPhys->getPosition();
				auto currpitch = atan2(currdir.y, 1.0f);
				auto tgtpitch = atan2(tgtvec.normalized().y, 1.0f) + Random::clamped(Random::SourceAI) * mVariation;
				auto diffpitch = -(currpitch - tgtpitch);
				mPhys->rotate(0.0f, 0.02f * diffpitch);
				if(fabs(diffyaw) < mVariation * 0.2f + 0.05f && fabs(diffpitch) < mVariation * 0.2f + 0.05f) {
					mShooter->shoot();
				}
			}
			break;
	}
}

float AIActor::turnTowards(const Common::Vector3& abspos, float variation)
{
	auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation());
	auto tgtvec = abspos - mPhys->getPosition();
	auto curryaw = atan2(currdir.z, currdir.x);
	auto tgtyaw = atan2(tgtvec.z, tgtvec.x);
	if(variation)
		tgtyaw += Random::clamped(Random::SourceAI) * variation;
	auto diffyaw = curryaw - tgtyaw;
	mPhys->rotate(0.02f * diffyaw, 0.0f);
	return diffyaw;
}


