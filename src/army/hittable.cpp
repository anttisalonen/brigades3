#include <sstream>

#include <common/Math.h>

#include "hittable.hpp"

HittableComponent::HittableComponent(const SoldierPhysics* phys, float radius, float height)
	: mPhys(phys),
	mRadius(radius),
	mHeight(height),
	mDied(false)
{
}

bool HittableComponent::hit(const Ray& ray) const
{
	auto ret = Common::Math::segmentSegmentDistance3D(ray.start, ray.end,
			mPhys->getPosition(),
			mPhys->getPosition() + Common::Vector3(0.0f, mHeight, 0.0f));
	return ret <= mRadius;
}

void HittableComponent::die()
{
	if(!mDied)
		std::cout << "Died!\n";
	mDied = true;
}

bool HittableComponent::hasDied() const
{
	return mDied;
}

HitterComponent::HitterComponent(const BulletPhysics* phys, unsigned int shooterID)
	: mPhys(phys),
	mShooterID(shooterID)
{
	mPrevPos = mThisPos = mPhys->getPosition();
}

void HitterComponent::update(float dt)
{
	// ensure the length of the ray isn't 0 or near 0 which might
	// lead to wrong result from
	// Common::Math::segmentCircleIntersect().
	auto newpos = mPhys->getPosition();
	if(mThisPos.distance2(newpos) > 0.1f) {
		mPrevPos = mThisPos;
		mThisPos = newpos;
	}
}

Ray HitterComponent::getLastRay() const
{
	Ray r;
	r.start = mPrevPos;
	r.end = mThisPos;
	return r;
}

unsigned int HitterComponent::getShooterID() const
{
	return mShooterID;
}

BulletRenderer::BulletRenderer()
{
}

BulletRenderer::BulletRenderer(Scene::Scene* scene, const HitterComponent* hit, unsigned int id)
	: mScene(scene),
	mHitter(hit)
{
	std::stringstream ss;
	ss << "Bullet line " << id;
	mName = ss.str();
}

void BulletRenderer::update(float dt)
{
	mScene->clearLine(mName);

	auto r = mHitter->getLastRay();
	if(mHitter->isActive() && r.start.distance2(r.end) > 1.0f) {
		mScene->addLine(mName, r.start, r.end, Common::Color::Red);
	}
}


