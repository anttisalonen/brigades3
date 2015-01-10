#include <assert.h>

#include <common/Math.h>

#include "random.hpp"
#include "house.hpp"

HouseWall::HouseWall(const Common::Vector2& start, const Common::Vector2& end, float width,
		float windowpos, float doorpos)
	: mStart(start),
	mEnd(end),
	mWidth(width),
	mWindowPos(windowpos),
	mDoorPos(doorpos)
{
}

const Common::Vector2& HouseWall::getStart() const
{
	return mStart;
}

const Common::Vector2& HouseWall::getEnd() const
{
	return mEnd;
}

float HouseWall::getWallHalfWidth() const
{
	return mWidth;
}

float HouseWall::getWindowPosition() const
{
	return mWindowPos;
}

float HouseWall::getDoorPosition() const
{
	return mDoorPos;
}

House::House(const Common::Vector3& p1,
		const Common::Vector3& p2,
		const Common::Vector3& p3,
		const Common::Vector3& p4,
		const std::vector<DoorInfo>& doors)
{
	const float halfwidth = 0.25f;
	mp[0] = Common::Vector2(p1.x, p1.z);
	mp[1] = Common::Vector2(p2.x, p2.z);
	mp[2] = Common::Vector2(p3.x, p3.z);
	mp[3] = Common::Vector2(p4.x, p4.z);

	// ensure the walls are attached together on the corners
	auto ext0 = (mp[0] - mp[1]).normalized() * halfwidth;
	auto ext1 = (mp[2] - mp[0]).normalized() * halfwidth;
	auto ext2 = (mp[1] - mp[3]).normalized() * halfwidth;
	auto ext3 = (mp[3] - mp[2]).normalized() * halfwidth;

	mFloor = p1.y;
	mRoof = mFloor + Random::uniform(Random::SourceWorld, 2.0f, 4.0f);

	mWalls.push_back(HouseWall(mp[0] + ext0, mp[1] - ext0, halfwidth, 0.0f, doors[0].DoorPosition));
	mWalls.push_back(HouseWall(mp[2] + ext1, mp[0] - ext1, halfwidth, 0.0f, 0.0f));
	mWalls.push_back(HouseWall(mp[1] + ext2, mp[3] - ext2, halfwidth, 0.0f, 0.0f));
	mWalls.push_back(HouseWall(mp[3] + ext3, mp[2] - ext3, halfwidth, 0.0f, 0.0f));
}

bool House::isInside(float x, float y) const
{
	Common::Vector2 p(x, y);
	return Common::Math::isInsideTriangle(p, mp[0], mp[1], mp[2]) ||
		Common::Math::isInsideTriangle(p, mp[2], mp[1], mp[3]);
}

float House::distanceTo(float x, float y) const
{
	if(isInside(x, y))
		return 0.0f;
	auto v = Common::Vector2(x, y);
	auto dist = mp[0].distance2(v);
	dist = std::min(dist, mp[1].distance2(v));
	dist = std::min(dist, mp[2].distance2(v));
	dist = std::min(dist, mp[3].distance2(v));
	return sqrt(dist);
}

float House::getHouseFloorHeight() const
{
	return mFloor;
}

float House::getHouseRoofHeight() const
{
	return mRoof;
}

const std::vector<HouseWall>& House::getWalls() const
{
	return mWalls;
}

Common::Vector3 House::getCornerstonePosition() const
{
	return Common::Vector3(mp[0].x, mFloor, mp[0].y);
}

std::pair<std::vector<Common::Vector3>, std::vector<Common::Vector3>> House::getVertexCoordsAndNormals() const
{
	std::pair<std::vector<Common::Vector3>, std::vector<Common::Vector3>> ret;
	auto roofHeight = mRoof - mFloor;

	std::vector<Common::Vector3> roofverts;
	std::vector<Common::Vector3> ceilverts;
	std::vector<Common::Vector3> floorverts;

	assert(mWalls.size() == 4); // due to roofverts & ceilverts

	for(const auto& wall : mWalls) {
		auto start = wall.getStart();
		auto end = wall.getEnd();
		auto halfwidth = wall.getWallHalfWidth();

		std::vector<Common::Vector2> norms = {
			Common::Math::rotate2D(end - start, HALF_PI).normalized() * halfwidth,
			Common::Math::rotate2D(end - start, -HALF_PI).normalized() * halfwidth,
		};

		for(unsigned int i = 0; i < 2; i++) {
			auto& norm = norms[i];
			auto p1 = Common::Vector3(start.x - mp[0].x + norm.x,
					0.0f,
					start.y - mp[0].y + norm.y);
			auto p2 = Common::Vector3(start.x - mp[0].x + norm.x,
					roofHeight,
					start.y - mp[0].y + norm.y);
			auto p3 = Common::Vector3(end.x - mp[0].x + norm.x,
					0.0f,
					end.y - mp[0].y + norm.y);
			auto p4 = Common::Vector3(end.x - mp[0].x + norm.x,
					roofHeight,
					end.y - mp[0].y + norm.y);
			if(i == 0) {
				ret.first.push_back(p3);
				ret.first.push_back(p4);
				ret.first.push_back(p1);
				ret.first.push_back(p2);

				ceilverts.push_back(p2 + Common::Vector3(0.0f, -0.1f, 0.0f));
			} else {
				ret.first.push_back(p1);
				ret.first.push_back(p2);
				ret.first.push_back(p3);
				ret.first.push_back(p4);

				roofverts.push_back(p4);
				floorverts.push_back(p1);
			}


			auto normal = p1.cross(p2);
			for(unsigned int i = 0; i < 4; i++) {
				(void)i;
				ret.second.push_back(normal);
			}
		}

	}

	for(unsigned int i = 0; i < 4; i++) {
		ret.first.push_back(roofverts[i]);
		ret.second.push_back(Common::Vector3(0.0f, 1.0f, 0.0f));
	}

	ret.first.push_back(ceilverts[0]);
	ret.first.push_back(ceilverts[2]);
	ret.first.push_back(ceilverts[1]);
	ret.first.push_back(ceilverts[3]);
	for(unsigned int i = 0; i < 4; i++) {
		ret.second.push_back(Common::Vector3(0.0f, -1.0f, 0.0f));
	}

	for(unsigned int i = 0; i < 4; i++) {
		ret.first.push_back(floorverts[i]);
		ret.second.push_back(Common::Vector3(0.0f, 1.0f, 0.0f));
	}

	return ret;
}

std::vector<Common::Vector2> House::getTexCoords() const
{
	std::vector<Common::Vector2> ret;
	for(const auto& wall : mWalls) {
		(void)wall;
		for(unsigned int i = 0; i < 2; i++) {
			(void)i;
			ret.push_back(Common::Vector2(0.0f, 0.0f));
			ret.push_back(Common::Vector2(0.0f, 1.0f));
			ret.push_back(Common::Vector2(1.0f, 0.0f));
			ret.push_back(Common::Vector2(1.0f, 1.0f));
		}
	}

	// roof, ceiling, floor
	for(unsigned int i = 0; i < 3; i++) {
		ret.push_back(Common::Vector2(0.0f, 0.0f));
		ret.push_back(Common::Vector2(0.0f, 1.0f));
		ret.push_back(Common::Vector2(1.0f, 0.0f));
		ret.push_back(Common::Vector2(1.0f, 1.0f));
	}

	return ret;
}

std::vector<unsigned int> House::getIndices() const
{
	std::vector<unsigned int> ret;
	unsigned int i = 0;
	for(const auto& wall : mWalls) {
		(void)wall;
		for(unsigned int j = 0; j < 2; j++) {
			(void)j;
			ret.push_back(i);
			ret.push_back(i + 1);
			ret.push_back(i + 2);
			ret.push_back(i + 2);
			ret.push_back(i + 1);
			ret.push_back(i + 3);
			i += 4;
		}
	}

	// roof, ceiling, floor
	for(unsigned int j = 0; j < 3; j++) {
		ret.push_back(i);
		ret.push_back(i + 1);
		ret.push_back(i + 2);
		ret.push_back(i + 2);
		ret.push_back(i + 1);
		ret.push_back(i + 3);
		i += 4;
	}

	return ret;
}


