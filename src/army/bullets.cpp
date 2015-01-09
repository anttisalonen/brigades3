#include <common/Math.h>
#include <common/Random.h>

#include "bullets.hpp"
#include "sound.hpp"

Bullets::Bullets(const WorldMap* wmap, Scene::Scene* scene)
	: mMap(wmap),
	mScene(scene),
	mNumBullets(0)
{
	mPhysics.resize(MAX_BULLETS);
	mHitters.resize(MAX_BULLETS);
	mRenders.resize(MAX_BULLETS);
}

void Bullets::update(float dt)
{
	for(unsigned int i = 0; i < mNumBullets; i++) {
		mPhysics[i].update(dt);
	}

	for(unsigned int i = 0; i < mNumBullets; i++) {
		mHitters[i].update(dt);
	}

	for(unsigned int i = 0; i < mNumBullets; i++) {
		mRenders[i].update(dt);
	}
}

void Bullets::shoot(Weapon& weapon, const Common::Vector3& pos, const Common::Quaternion& ori, unsigned int shooterID)
{
	assert(mNumBullets < MAX_BULLETS);
	BulletPhysics* ph = &mPhysics[mNumBullets];
	*ph = BulletPhysics(mMap, 0.0f, 0.99f, 0.0f);
	ph->setPosition(pos + Common::Vector3(0.0f, 1.7f, 0.0f));
	auto velvec = Common::Math::rotate3D(Scene::WorldForward, ori);
	velvec.x += Common::Random::clamped() * 0.001f;
	velvec.y += Common::Random::clamped() * 0.001f;
	velvec.z += Common::Random::clamped() * 0.001f;
	velvec.normalize();
	ph->setVelocity(velvec * 700.0f);

	mHitters[mNumBullets] = HitterComponent(ph, shooterID);
	mRenders[mNumBullets] = BulletRenderer(mScene, &mHitters[mNumBullets], mNumBullets);

	mNumBullets++;
	weapon.shoot();
	Sound::play(ph->getPosition() + velvec);
}

std::vector<unsigned int> Bullets::checkForHits(const std::vector<HittableComponent>& hittables)
{
	std::vector<unsigned int> ret;
	for(unsigned int j = 0; j < hittables.size(); j++) {
		for(unsigned int i = 0; i < mNumBullets; i++) {
			if(!mHitters[i].isActive())
				continue;

			auto r = mHitters[i].getLastRay();
			if(mHitters[i].getShooterID() != j && hittables[j].hit(r)) {
				ret.push_back(j);
				break;
			}
		}
	}
	return ret;
}

ShooterComponent::ShooterComponent(const SoldierPhysics* phys, Bullets* bullets, unsigned int shooterID)
	: mPhys(phys),
	mBullets(bullets),
	mShooterID(shooterID)
{
}

void ShooterComponent::shoot()
{
	if(mWeapon.canShoot()) {
		mBullets->shoot(mWeapon, mPhys->getPosition(), mPhys->getOrientation() * mPhys->getAimPitch(), mShooterID);
	}
}

void ShooterComponent::update(float dt)
{
	mWeapon.update(dt);
}


