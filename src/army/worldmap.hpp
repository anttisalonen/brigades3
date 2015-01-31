#ifndef BRIGADES_WORLDMAP_HPP
#define BRIGADES_WORLDMAP_HPP

#include <vector>

#include <common/Vector2.h>
#include <common/Vector3.h>

#include <sscene/Scene.h>

#include <noise/noise.h>

#include "random.hpp"
#include "house.hpp"

struct Tree {
	Tree(const Common::Vector3& pos, float radius);
	Common::Vector3 Position;
	float Radius;
	static constexpr float UNPASSABLE_COEFFICIENT   = 0.6f;
	static constexpr float UNSEETHROUGH_COEFFICIENT = 0.7f;
	static constexpr float TRUNK_COEFFICIENT        = 0.1f;
	static constexpr float HEIGHT_COEFFICIENT       = 2.0f;
	static constexpr float BULLET_SLOWDOWN_PER_TREE_METER = 1000.0f;
};

struct Ray {
	Common::Vector3 start;
	Common::Vector3 end;
};

class WorldMap {
	public:
		WorldMap(Scene::Scene* scene);
		void create();
		float getHeightAt(float x, float y) const;
		std::vector<Tree> getTreesAt(float x, float y, float r) const;
		Common::Vector3 getNormalAt(float x, float y) const;
		float lineBlockedByObstacles(const Common::Vector3& p1, const Common::Vector3& p2, bool bullet, Common::Vector3* nearest) const;
		bool lineBlockedByLand(const Common::Vector3& p1, const Common::Vector3& p2, Common::Vector3* hit) const;
		float lineBlockedByTrees(const Common::Vector3& p1, const Common::Vector3& p2, bool bullet, Common::Vector3* nearest) const;
		float getWidth() const;
		Common::Vector3 findFreeSpot(unsigned int tries, Random::Source rs) const;
		const std::vector<House>& getHousesAt(float x, float y, float r) const;
		Common::Vector3 pointToVec(float x, float y) const;
		float getHeightOnCollisionPoint(const Common::Vector3& p1, const Common::Vector3& p2,
				const Common::Vector2& collpoint) const;
		float lineBlockedByWalls(const Common::Vector3& p1, const Common::Vector3& p2,
				Common::Vector3* nearest, HouseWall* hitwall = nullptr) const;

	private:
		void addHouses();
		void addTrees();
		bool nearHouse(float x, float y, float radius, House* house) const;
		bool isFreeSpot(float x, float y) const;

		Scene::Scene* mScene;
		std::vector<Tree> mTrees;

		noise::module::Perlin mNoise;

		std::vector<House> mHouses;
};

#endif
