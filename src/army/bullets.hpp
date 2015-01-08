#ifndef BRIGADES_BULLETS_HPP
#define BRIGADES_BULLETS_HPP

#include <vector>

#include <common/Vector3.h>
#include <common/Quaternion.h>

#include <sscene/Scene.h>

#include "worldmap.hpp"
#include "weapon.hpp"
#include "bulletphysics.hpp"
#include "hittable.hpp"

class Bullets {
	static const unsigned int MAX_BULLETS = 256;
	public:
		Bullets(const WorldMap* wmap, Scene::Scene* scene);
		void shoot(Weapon& weapon, const Common::Vector3& pos, const Common::Quaternion& ori, unsigned int shooterID);
		void update(float dt);
		std::vector<unsigned int> checkForHits(const std::vector<HittableComponent>& hittables);

	private:
		std::vector<BulletPhysics> mPhysics;
		std::vector<HitterComponent> mHitters;
		std::vector<BulletRenderer> mRenders;
		const WorldMap* mMap;
		Scene::Scene* mScene;
		unsigned int mNumBullets;
};

class ShooterComponent {
	public:
		ShooterComponent(const SoldierPhysics* phys, Bullets* bullets, unsigned int shooterID);
		ShooterComponent() = default;
		void shoot();
		void update(float dt);

	private:
		const SoldierPhysics* mPhys;
		Bullets* mBullets;
		Weapon mWeapon;
		unsigned int mShooterID;
};

#endif
