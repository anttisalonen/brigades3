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
	PhysicsCommon<SoldierPhysics>::update(dt, *this);
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

void SoldierPhysics::addAcceleration(const Common::Vector3& vec)
{
	if(mAiming)
		return;

	mAcceleration += vec;
}

Common::Quaternion SoldierPhysics::getAimPitch() const
{
	return Common::Quaternion::fromAxisAngle(Common::Vector3(0.0f, 0.0f, 1.0f), mAimPitch);
}

void SoldierPhysics::rotate(float yaw, float pitch)
{
	mOrientation = mOrientation *
		Common::Quaternion::fromAxisAngle(Common::Vector3(0.0f, 1.0f, 0.0f), yaw);
	mAimPitch = Common::clamp<float>(-HALF_PI * 0.6f, mAimPitch + pitch, HALF_PI * 0.6f);
}

void SoldierPhysics::setAiming(bool a)
{
	mAiming = a;
}

bool SoldierPhysics::isAiming() const
{
	return mAiming;
}


