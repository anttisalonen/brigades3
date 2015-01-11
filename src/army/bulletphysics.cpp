#include "bulletphysics.hpp"

BulletPhysics::BulletPhysics(const WorldMap* wmap, float maxvel, float damping, float friction)
	: mMap(wmap),
	mMaxVel(maxvel),
	mDamping(damping),
	mActive(true)
{
}

void BulletPhysics::update(float dt)
{
	if(mVelocity.length() < 1.0f) {
		mActive = false;
	}

	if(!mActive)
		return;

	PhysicsCommon<BulletPhysics>::update(dt, *this);
}

void BulletPhysics::checkLandCollision(const Common::Vector3& oldpos)
{
	Common::Vector3 hitpoint;
	if(mMap->lineBlockedByLand(oldpos, mPosition, &hitpoint)) {
		mPosition = hitpoint;
		mVelocity.zero();
	}
}

void BulletPhysics::checkWallCollision(const Common::Vector3& oldpos)
{
	Common::Vector3 nearest;
	auto ret = mMap->lineBlockedByWalls(oldpos, mPosition, &nearest);
	if(ret) {
		mPosition = nearest;
		mVelocity.zero();
	}
}

void BulletPhysics::checkTreeCollision(const Common::Vector3& oldpos)
{
	Common::Vector3 nearest;
	auto obscoeff = mMap->lineBlockedByTrees(oldpos, mPosition, true, &nearest);
	if(obscoeff) {
		auto currSpeed = mVelocity.length();
		auto speed = std::max<double>(0.0,
				currSpeed - obscoeff);
		mVelocity = mVelocity.normalized() * speed;

		if(mVelocity.length() < 1.0f) {
			mPosition = nearest;
		}
	}
}


