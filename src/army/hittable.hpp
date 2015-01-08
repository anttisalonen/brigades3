#ifndef BRIGADES_HITTABLE_HPP
#define BRIGADES_HITTABLE_HPP

#include <string>

#include <common/Vector3.h>

#include <sscene/Scene.h>

#include "soldierphysics.hpp"
#include "bulletphysics.hpp"

class HittableComponent {
	public:
		HittableComponent() = default;
		HittableComponent(const SoldierPhysics* phys, float radius, float height);
		bool hit(const Ray& ray) const;
		void die();
		bool hasDied() const;

	private:
		const SoldierPhysics* mPhys;
		float mRadius;
		float mHeight;
		bool mDied;
};

class HitterComponent {
	public:
		HitterComponent() = default;
		HitterComponent(const BulletPhysics* phys, unsigned int shooterID);
		void update(float dt);
		Ray getLastRay() const;
		bool isActive() const { return mPhys->isActive(); }
		unsigned int getShooterID() const;

	private:
		const BulletPhysics* mPhys;
		Common::Vector3 mPrevPos;
		Common::Vector3 mThisPos;
		unsigned int mShooterID;
};

class BulletRenderer {
	public:
		BulletRenderer();
		BulletRenderer(Scene::Scene* scene, const HitterComponent* hit, unsigned int id);
		void update(float dt);

	private:
		Scene::Scene* mScene;
		const HitterComponent* mHitter;
		std::string mName;
};

#endif

