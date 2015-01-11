#include <common/Math.h>

#include "soldierphysics.hpp"

SoldierPhysics::SoldierPhysics(const WorldMap* wmap, float maxvel, float damping, float friction)
	: mAimPitch(0.0f),
	mMap(wmap),
	mMaxVel(maxvel),
	mDamping(damping),
	mFriction(friction),
	mAiming(false)
{
}

void SoldierPhysics::update(float dt)
{
	mAcceleration += mMovementAcceleration;
	mMovementAcceleration.zero();
	PhysicsCommon<SoldierPhysics>::update(dt, *this);

	mYawVelocity = Common::clamp(-10.0f, mYawVelocity, 10.0f);
	mPitchVelocity = Common::clamp(-10.0f, mPitchVelocity, 10.0f);
	mOrientation = mOrientation *
		Common::Quaternion::fromAxisAngle(Common::Vector3(0.0f, 1.0f, 0.0f), mYawVelocity * dt);
	mAimPitch = Common::clamp<float>(-HALF_PI * 0.6f, mAimPitch + mPitchVelocity * dt, HALF_PI * 0.6f);

	mYawVelocity = 0.0f;
	mPitchVelocity = 0.0f;
}

void SoldierPhysics::checkLandCollision(const Common::Vector3& oldpos)
{
	float hgt = mMap->getHeightAt(mPosition.x, mPosition.z);
	float ydiff = hgt - mPosition.y;
	if(ydiff > 0.0f) {
		mPosition.y += ydiff;
		mVelocity = mVelocity * mFriction;
		mVelocity.y = 0.0f;
	}

	if(hgt < -1.2f) {
		auto oldhgt = mMap->getHeightAt(oldpos.x, oldpos.z);
		if(oldhgt > hgt) {
			mPosition = oldpos;
		}
	}
}

void SoldierPhysics::checkTreeCollision(const Common::Vector3& oldpos)
{
	for(const auto& t : mMap->getTreesAt(mPosition.x, mPosition.z, 5.0f)) {
		auto rad = t.Radius * Tree::UNPASSABLE_COEFFICIENT;
		auto rad2 = rad * rad;
		if(t.Position.distance2(mPosition) < rad2) {
			auto diff = (mPosition - t.Position).normalized();
			auto newpos = t.Position + diff * rad;
			mPosition = newpos;
		}
	}
}

void SoldierPhysics::checkWallCollision(const Common::Vector3& oldpos)
{
	auto travdist = oldpos.distance(mPosition);
	for(const auto& house : mMap->getHousesAt(mPosition.x, mPosition.z, 5.0f)) {
		for(const auto& wall : house.getWalls()) {
			const auto& rs = wall.getStart();
			const auto& re = wall.getEnd();
			float width = wall.getWallHalfWidth();
			Common::Vector2 mp = Common::Vector2(mPosition.x, mPosition.z);
			Common::Vector2 op = Common::Vector2(oldpos.x, oldpos.z);

			Common::Vector2 nearest;

			auto dist = Common::Math::pointToSegmentDistance(
					rs,
					re,
					mp, &nearest);
			if(dist < width + 1.0f) {
				// soldier is within wall
				auto vecFromWall = nearest +
					(mp - nearest).normalized() * (width + 1.0f);
				mPosition = mMap->pointToVec(vecFromWall.x, vecFromWall.y);
			} else if(travdist > width) {
				// not within wall but possibly tunneled
				bool found;
				auto v = Common::Math::segmentSegmentIntersection2D(
						rs,
						re,
						op,
						mp,
						&found);
				if(found) {
					std::cout << "Soldier tunneled through wall " << rs << "\t" << re << "\n";
					std::cout << "Dist: " << dist << "\toldpos: " << op << "\t" << mp << "\n";
					std::cout << "v: " << v << "\n";
					mPosition = oldpos;
				}
			}
		}
	}
}

void SoldierPhysics::addMovementAcceleration(const Common::Vector3& vec)
{
	if(mAiming)
		return;

	mMovementAcceleration.x += vec.x;
	mMovementAcceleration.y = 0.0f;
	mMovementAcceleration.z += vec.z;
	mMovementAcceleration.truncate(RunAcceleration);
}

Common::Quaternion SoldierPhysics::getAimPitch() const
{
	return Common::Quaternion::fromAxisAngle(Common::Vector3(0.0f, 0.0f, 1.0f), mAimPitch);
}

void SoldierPhysics::addRotation(float yaw, float pitch)
{
	mYawVelocity += yaw * 60.0f;
	mPitchVelocity += pitch * 60.0f;
}

void SoldierPhysics::setAiming(bool a)
{
	mAiming = a;
}

bool SoldierPhysics::isAiming() const
{
	return mAiming;
}


