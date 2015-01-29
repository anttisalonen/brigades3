#include <float.h>

#include <sstream>

#include <common/Math.h>

#include "worldmap.hpp"

class Heightmap : public Scene::Heightmap {
	public:
		Heightmap(unsigned int w, float xzscale, std::function<float (float, float)> func);
		virtual float getHeightAt(float x, float y) const;
		virtual unsigned int getWidth() const;
		virtual float getXZScale() const;

	private:
		std::function<float (float, float)> mHeightFunc;
		unsigned int mWidth;
		float mXZScale;
};

Heightmap::Heightmap(unsigned int w, float xzscale, std::function<float (float, float)> func)
	: mHeightFunc(func),
	mWidth(w),
	mXZScale(xzscale)
{
}

float Heightmap::getHeightAt(float x, float y) const
{
	return mHeightFunc(x, y);
}

unsigned int Heightmap::getWidth() const
{
	return mWidth;
}

float Heightmap::getXZScale() const
{
	return mXZScale;
}

Tree::Tree(const Common::Vector3& pos, float radius)
	: Position(pos),
	Radius(radius)
{
}

WorldMap::WorldMap(Scene::Scene* scene)
	: mScene(scene)
{
}

float WorldMap::getWidth() const
{
	return 256.0f;
}

bool WorldMap::isFreeSpot(float x, float y) const
{
	return getHeightAt(x, y) > 0.0f && !nearHouse(x, y, 10.0f, nullptr);
}

Common::Vector3 WorldMap::findFreeSpot(unsigned int tries, Random::Source rs) const
{
	auto w = getWidth();

	for(unsigned int i = 0; i < tries; i++) {
		float x = Random::uniform(rs) * w;
		float y = Random::uniform(rs) * w;
		if(isFreeSpot(x, y)) {
			return pointToVec(x, y);
		}
	}
	return Common::Vector3();
}

bool WorldMap::nearHouse(float x, float y, float radius, House* house) const
{
	// NOTE: radius must be more than half of maximum house wall length
	for(const auto& h : getHousesAt(x, y, radius)) {
		if(h.distanceTo(x, y) < radius) {
			if(house)
				*house = h;
			return true;
		}
	}
	return false;
}

void WorldMap::create()
{
	mNoise.SetSeed(Random::uniform(Random::SourceWorld, 0, static_cast<unsigned int>(INT_MAX)));
	addHouses();
	addTrees();

	if(mScene) {
		auto hmapconv = getWidth() / 128.0f;
		Heightmap hm(128, hmapconv, [&] (float x, float y) { return getHeightAt(x, y); });

		mScene->addModelFromHeightmap("Terrain", hm);
		auto mi = mScene->addMeshInstance("Terrain", "Terrain", "Snow");

		mScene->addPlane("Sea", getWidth(), getWidth(), 16);
		auto mi2 = mScene->addMeshInstance("Sea", "Sea", "Sea");
		mi2->setPosition(Common::Vector3(-getWidth() / 2.0f, 0.0f, -getWidth() / 2.0f));
		mi2->setScale(getWidth() * 2.0f, 1.0f, getWidth() * 2.0f);
	}
}

Common::Vector3 WorldMap::pointToVec(float x, float y) const
{
	return Common::Vector3(x, getHeightAt(x, y), y);
}


void WorldMap::addHouses()
{
	for(int i = 0; i < 50; i++) {
		Common::Vector3 cornerstone = findFreeSpot(100, Random::SourceWorld);
		if(cornerstone.null())
			break;

		auto corner2x = Random::uniform(Random::SourceWorld, 5.0f, 15.0f);
		auto corner2z = Random::uniform(Random::SourceWorld, 5.0f, 15.0f);
		auto corner2 = pointToVec(cornerstone.x + corner2x,
				cornerstone.z + corner2z);

		if(!isFreeSpot(corner2.x, corner2.z)) {
			continue;
		}

		auto corner3_2d = Common::Math::rotate2D(Common::Vector2(corner2x,
					corner2z), HALF_PI);

		auto corner3x = Random::uniform(Random::SourceWorld, 5.0f, 15.0f);

		corner3_2d = corner3_2d.normalized() * corner3x;
		Common::Vector3 corner3_rel(corner3_2d.x, 0.0f, corner3_2d.y);
		Common::Vector3 corner3 = corner3_rel + cornerstone;
		if(!isFreeSpot(corner3.x, corner3.z)) {
			continue;
		}
		corner3.y = getHeightAt(corner3.x, corner3.z);

		Common::Vector3 corner4 = corner2 + corner3_rel;
		if(!isFreeSpot(corner4.x, corner4.z)) {
			continue;
		}
		corner4.y = getHeightAt(corner4.x, corner4.z);

		auto minmaxheights = std::minmax({cornerstone.y,
				corner2.y, corner3.y,
				corner4.y});

		if(minmaxheights.second - minmaxheights.first > 1.0f) {
			continue;
		}

		if(minmaxheights.first < 1.0f) {
			continue;
		}

		cornerstone.y = corner2.y = corner4.y = minmaxheights.first;

		auto house = House(cornerstone, corner2,
				corner3,
				corner4, {{0, 0.5f, 1.0f}});
		mHouses.push_back(house);
	}

	if(mScene) {
		for(unsigned int i = 0; i < mHouses.size(); i++) {
			std::stringstream ss;
			ss << "House " << i << "\n";
			std::string hname = ss.str();
			auto vpair = mHouses[i].getVertexCoordsAndNormals();
			auto texcoords = mHouses[i].getTexCoords();
			auto indices = mHouses[i].getIndices();
			mScene->addModel(hname, vpair.first, texcoords, indices, vpair.second);
			auto mi = mScene->addMeshInstance(hname, hname, "House");
			auto cp = mHouses[i].getCornerstonePosition();
			std::cout << "Cornerstone at " << cp << "\n";
			mi->setPosition(cp);
		}
	}
}

void WorldMap::addTrees()
{
	// TODO: Create a single mesh instance with all models instead
	for(int i = 0; i < 500; i++) {
		auto pos = findFreeSpot(1, Random::SourceWorld);
		if(!pos.null()) {
			float r = Random::uniform(Random::SourceWorld, 2.0f, 5.0f);
			float rot = Random::uniform(Random::SourceWorld, 0.0f, QUARTER_PI);
			mTrees.push_back(Tree(pos, r));

			if(mScene) {
				char name[256];
				snprintf(name, 255, "Tree%d", i);

				auto treeInst = mScene->addMeshInstance(name, "Tree", "Tree", false, true);
				treeInst->setPosition(pos);
				treeInst->setScale(r, r, r);
				treeInst->setRotationFromEuler(Common::Vector3(0.0f, rot, 0.0f));
			}
		}
	}
}

float WorldMap::getHeightAt(float x, float y) const
{
	float w = getWidth();
	if(x < 0 || x > w || y < 0 || y > w)
		return -5.0f;

	{
		House house;
		if(nearHouse(x, y, 8.0f, &house)) {
			return house.getHouseFloorHeight() - 0.05f;
		}
	}

	const float coast = 32.0f;
	float cdiff = FLT_MAX;
	if(x > w - coast)
		cdiff = w - x;
	if(x < coast)
		cdiff = std::min(cdiff, x);
	if(y > w - coast)
		cdiff = std::min(cdiff, w - y);
	if(y < coast)
		cdiff = std::min(cdiff, y);

	// TODO: lerp points sampled by gfx instead
	float hval = ((mNoise.GetValue(x * 0.01f, 0.0, y * 0.01f) + 1.0f) * 0.5f) * 10.0f - 2.0f;

	if(cdiff < coast) {
		// lerp
		return (-5.0f * (coast - cdiff) / coast) + (hval * cdiff / coast);
	} else {
		return hval;
	}
}

std::vector<Tree> WorldMap::getTreesAt(float x, float y, float r) const
{
	// TODO: This can be optimised
	return mTrees;
}

const std::vector<House>& WorldMap::getHousesAt(float x, float y, float r) const
{
	return mHouses;
}

Common::Vector3 WorldMap::getNormalAt(float x, float y) const
{
	Common::Vector3 p1 = pointToVec(x, y);
	Common::Vector3 p2 = pointToVec(x + 0.5f, y);
	Common::Vector3 p3 = pointToVec(x, y + 0.5f);
	Common::Vector3 u(p2 - p1);
	Common::Vector3 v(p3 - p1);
	return v.cross(u).normalized();
}

bool WorldMap::lineBlockedByLand(const Common::Vector3& p1, const Common::Vector3& p2, Common::Vector3* hit) const
{
	float distBetweenSamples = std::max(1.0f, getHeightAt(p1.x, p1.z) - p1.y);
	float fullDist = p1.distance(p2);
	auto dir = (p2 - p1).normalized();

	for(float sampleDist = 0.0f;
			sampleDist < fullDist;
			sampleDist += distBetweenSamples) {
		auto samplePos = p1 + dir * sampleDist;
		float hgt = getHeightAt(samplePos.x, samplePos.z);
		if(hgt > samplePos.y) {
			if(hit)
				*hit = samplePos;
			return true;
		}
		distBetweenSamples = std::max(1.0f, hgt - samplePos.y);
	}
	return false;
}

float WorldMap::lineBlockedByObstacles(const Common::Vector3& p1, const Common::Vector3& p2, bool bullet, Common::Vector3* nearest) const
{
	Common::Vector3 nearestTree;
	Common::Vector3 nearestWall;
	auto treeRet = lineBlockedByTrees(p1, p2, bullet, &nearestTree);
	auto wallRet = lineBlockedByWalls(p1, p2, &nearestWall);
	if(treeRet && wallRet) {
		if(p1.distance2(nearestTree) < p1.distance2(nearestWall)) {
			if(nearest)
				*nearest = nearestTree;
			return treeRet;
		} else {
			if(nearest)
				*nearest = nearestWall;
			return wallRet;
		}
	} else if(treeRet) {
		if(nearest)
			*nearest = nearestTree;
		return treeRet;
	} else if(wallRet) {
		if(nearest)
			*nearest = nearestWall;
		return wallRet;
	}

	return 0.0f;
}

float WorldMap::lineBlockedByTrees(const Common::Vector3& p1, const Common::Vector3& p2, bool bullet, Common::Vector3* nearest) const
{
	float obscoeff = 0.0f;
	auto treeBlockCoeff = bullet ? Tree::TRUNK_COEFFICIENT : Tree::UNSEETHROUGH_COEFFICIENT;
	auto trees = getTreesAt(p2.x, p2.z, p1.distance(p2));

	if(nearest)
		*nearest = Common::Vector3();

	std::sort(trees.begin(), trees.end(), [&] (const Tree& t1, const Tree& t2) {
			return p1.distance2(t1.Position) <
				p1.distance2(t2.Position); });

	for(const auto& t : trees) {
		auto treeHeight = t.Position.y + t.Radius * Tree::HEIGHT_COEFFICIENT;
		if(p2.y > treeHeight && p2.y > treeHeight)
			continue;

		Common::Vector2 nearest2;
		Common::Vector2 p12(p1.x, p1.z);
		Common::Vector2 p22(p2.x, p2.z);
		auto dist = Common::Math::pointToSegmentDistance(p12,
				p22,
				Common::Vector2(t.Position.x, t.Position.z), &nearest2);
		auto rad = t.Radius * treeBlockCoeff;

		if(dist < rad) {
			if(!bullet) {
				return 1.0f;
			} else {
				auto distTravelledInTrunk = rad - dist;
				obscoeff += distTravelledInTrunk * Tree::BULLET_SLOWDOWN_PER_TREE_METER;
				if(nearest && nearest->null()) {
					auto newHeight = getHeightOnCollisionPoint(p1,
							p2,
							nearest2);
					*nearest = Common::Vector3(nearest2.x, newHeight, nearest2.y);
				}
			}
		}
	}
	return obscoeff;
}

float WorldMap::lineBlockedByWalls(const Common::Vector3& p1, const Common::Vector3& p2,
		Common::Vector3* nearest, HouseWall* hitwall) const
{
	// TODO: this doesn't check for collision against the roof
	Common::Vector2 mp = Common::Vector2(p2.x, p2.z);
	Common::Vector2 op = Common::Vector2(p1.x, p1.z);

	float distToNearest = FLT_MAX;

	for(const auto& house : getHousesAt(p2.x, p2.z, p1.distance(p2) + 5.0f)) {
		auto roofheight = house.getHouseRoofHeight();
		if(roofheight < p2.y && roofheight < p1.y)
			continue;

		for(const auto& wall : house.getWalls()) {
			const auto& rs = wall.getStart();
			const auto& re = wall.getEnd();

			bool found;
			auto hitpoint = Common::Math::segmentSegmentIntersection2D(
					rs,
					re,
					op,
					mp,
					&found);
			if(found) {
				auto newHeight = getHeightOnCollisionPoint(p1,
						p2,
						hitpoint);
				if(newHeight < roofheight) {
					auto hitpoint3 = Common::Vector3(hitpoint.x, newHeight, hitpoint.y);
					auto thisDist = p1.distance2(hitpoint3);
					if(thisDist < distToNearest) {
						distToNearest = thisDist;
						if(nearest)
							*nearest = hitpoint3;
						if(hitwall)
							*hitwall = wall;
					}
				}
			}
		}
	}

	if(distToNearest != FLT_MAX)
		return 1.0f;
	return 0.0f;
}

float WorldMap::getHeightOnCollisionPoint(const Common::Vector3& p1, const Common::Vector3& p2,
		const Common::Vector2& collpoint) const
{
	auto distFromStartToImpact = Common::Vector2(p1.x, p1.z).distance(collpoint);
	auto distFromEndToImpact = Common::Vector2(p2.x, p2.z).distance(collpoint);
	auto distCoeff = distFromStartToImpact / (distFromStartToImpact + distFromEndToImpact);
	return p1.y + distCoeff * (p2.y - p1.y);
}


