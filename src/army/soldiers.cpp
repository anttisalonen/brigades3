#include "soldiers.hpp"

RenderComponent::RenderComponent(Scene::Scene& scene,
		const SoldierPhysics* phys,
		const HittableComponent* hit,
		unsigned int num)
	: mPhys(phys),
	mHit(hit)
{
	char name[256];
	snprintf(name, 255, "Soldier%d", num);
	mMesh = scene.addMeshInstance(name, "Soldier", "Soldier").get();
}

void RenderComponent::update(float dt)
{
	mMesh->setPosition(mPhys->getPosition());
	if(mHit->hasDied()) {
		// TODO: This could be animated
		// TODO: The angle should correspond to the ground
		mMesh->setRotation(mPhys->getOrientation() * Common::Quaternion(0.0f, 0.0f, sqrt(0.5f), sqrt(0.5f)));
	} else {
		mMesh->setRotation(mPhys->getOrientation());
	}
}

Soldiers::Soldiers(Scene::Scene& scene)
	: mNumSoldiers(0),
	mScene(scene),
	mPlayerSoldierIndex(0)
{
	mPhysics.resize(MAX_SOLDIERS);
	mRenders.resize(MAX_SOLDIERS);
	mShooters.resize(MAX_SOLDIERS);
	mHittables.resize(MAX_SOLDIERS);
}

void Soldiers::update(float dt)
{
	for(unsigned int i = 0; i < mNumSoldiers; i++) {
		mPhysics[i].update(dt);
	}
	for(unsigned int i = 0; i < mNumSoldiers; i++) {
		mShooters[i].update(dt);
	}
	for(unsigned int i = 0; i < mNumSoldiers; i++) {
		mRenders[i].update(dt);
	}
}

void Soldiers::addSoldiers(const WorldMap* wmap, Bullets* bullets, unsigned int numSoldiers)
{
	mPlayerSoldierIndex = 0;
	std::default_random_engine gen((unsigned int)time(0));
	for(unsigned int i = 0; i < numSoldiers; i++) {
		mPhysics[i] = SoldierPhysics(wmap, 5.0f, 0.10f, 1.0f);
		auto pos = wmap->findFreeSpot(100, gen);
		if(pos.null()) {
			throw std::runtime_error("No space for a soldier in the world");
		}
		mPhysics[i].setPosition(pos);
		mShooters[i] = ShooterComponent(&mPhysics[i], bullets, i);
		mHittables[i] = HittableComponent(&mPhysics[i], 0.3f, 1.7f);
		mRenders[i] = RenderComponent(mScene, &mPhysics[i], &mHittables[i], i);
	}

	mNumSoldiers = numSoldiers;
}

const Common::Vector3& Soldiers::getPlayerSoldierPosition() const
{
	assert(mPlayerSoldierIndex < mNumSoldiers);

	return mPhysics[mPlayerSoldierIndex].getPosition();
}

Common::Quaternion Soldiers::getPlayerSoldierOrientation() const
{
	assert(mPlayerSoldierIndex < mNumSoldiers);

	return mPhysics[mPlayerSoldierIndex].getOrientation() * mPhysics[mPlayerSoldierIndex].getAimPitch();
}

bool Soldiers::getPlayerSoldierAiming() const
{
	return mPhysics[mPlayerSoldierIndex].isAiming();
}

SoldierPhysics* Soldiers::getPlayerPhysics()
{
	return getPhys(mPlayerSoldierIndex);
}

ShooterComponent* Soldiers::getPlayerShooter()
{
	return getShooter(mPlayerSoldierIndex);
}

HittableComponent* Soldiers::getPlayerHittable()
{
	return getHittable(mPlayerSoldierIndex);
}

SoldierPhysics* Soldiers::getPhys(unsigned int i)
{
	assert(i < mNumSoldiers);
	return &mPhysics[i];
}

const SoldierPhysics* Soldiers::getPhys(unsigned int i) const
{
	assert(i < mNumSoldiers);
	return &mPhysics[i];
}

const Common::Vector3& Soldiers::getSoldierPosition(unsigned int id) const
{
	return getPhys(id)->getPosition();
}

ShooterComponent* Soldiers::getShooter(unsigned int i)
{
	assert(i < mNumSoldiers);
	return &mShooters[i];
}

HittableComponent* Soldiers::getHittable(unsigned int i)
{
	assert(i < mNumSoldiers);
	return &mHittables[i];
}

std::vector<HittableComponent> Soldiers::getHittables() const
{
	return std::vector<HittableComponent>(mHittables.begin(), mHittables.begin() + mNumSoldiers);
}

void Soldiers::processHits(const std::vector<unsigned int>& hits)
{
	for(auto i : hits) {
		mHittables[i].die();
	}
}

unsigned int Soldiers::getNumSoldiers() const
{
	return mNumSoldiers;
}

unsigned int Soldiers::getPlayerSoldierIndex() const
{
	return mPlayerSoldierIndex;
}

std::vector<unsigned int> Soldiers::getSoldiersAt(const Common::Vector3& pos, float radius) const
{
	// TODO: improve
	std::vector<unsigned int> ret;
	for(unsigned int i = 0; i < mNumSoldiers; i++) {
		ret.push_back(i);
	}
	return ret;
}

bool Soldiers::soldierIsAlive(unsigned int i) const
{
	return !mHittables[i].hasDied();
}


