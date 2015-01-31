#ifndef BRIGADES_SOLDIERS_HPP
#define BRIGADES_SOLDIERS_HPP

#include <common/Vector3.h>

#include <sscene/Scene.h>

#include "soldierphysics.hpp"
#include "hittable.hpp"
#include "worldmap.hpp"
#include "bullets.hpp"

class RenderComponent {
	public:
		RenderComponent(Scene::Scene* scene,
				const SoldierPhysics* phys,
				const HittableComponent* hit,
				unsigned int num);
		RenderComponent() = default;
		void update(float dt);

	private:
		const SoldierPhysics* mPhys;
		const HittableComponent* mHit;
		Scene::MeshInstance* mMesh;
};

class Soldiers {
	public:
		Soldiers(Scene::Scene* scene, unsigned int playerSoldierIndex);
		void update(float dt);
		void addSoldiers(const WorldMap* wmap, Bullets* bullets, unsigned int numSoldiers);
		const Common::Vector3& getPlayerSoldierPosition() const;
		Common::Quaternion getPlayerSoldierOrientation() const;
		Common::Quaternion getSoldierOrientation(unsigned int i) const;
		bool getPlayerSoldierAiming() const;
		bool getSoldierAiming(unsigned int i) const;
		SoldierPhysics* getPlayerPhysics();
		ShooterComponent* getPlayerShooter();
		HittableComponent* getPlayerHittable();
		SoldierPhysics* getPhys(unsigned int i);
		const SoldierPhysics* getPhys(unsigned int i) const;
		const Common::Vector3& getSoldierPosition(unsigned int id) const;
		const Common::Vector3& getSoldierVelocity(unsigned int id) const;
		ShooterComponent* getShooter(unsigned int i);
		HittableComponent* getHittable(unsigned int i);
		std::vector<HittableComponent> getHittables() const;
		void processHits(const std::vector<unsigned int>& hits);
		unsigned int getNumSoldiers() const;
		unsigned int getPlayerSoldierIndex() const;
		std::vector<unsigned int> getSoldiersAt(const Common::Vector3& pos, float radius) const;
		bool soldierIsAlive(unsigned int i) const;
		const std::string& getName(unsigned int i) const;

		static const unsigned int MAX_SOLDIERS = 256;

	private:
		unsigned int mNumSoldiers;
		std::vector<SoldierPhysics> mPhysics;
		std::vector<RenderComponent> mRenders;
		std::vector<ShooterComponent> mShooters;
		std::vector<HittableComponent> mHittables;
		std::vector<std::string> mSoldierNames;

		Scene::Scene* mScene;
		unsigned int mPlayerSoldierIndex;
};

#endif

