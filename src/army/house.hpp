#ifndef BRIGADES_HOUSE_HPP
#define BRIGADES_HOUSE_HPP

#include <vector>
#include <array>

#include <common/Vector2.h>
#include <common/Vector3.h>

class HouseWall {
	public:
		HouseWall(const Common::Vector2& start, const Common::Vector2& end, float halfwidth,
				float windowpos, float doorpos);
		const Common::Vector2& getStart() const;
		const Common::Vector2& getEnd() const;
		float getWallHalfWidth() const;
		float getWindowPosition() const;
		float getDoorPosition() const;

	private:
		Common::Vector2 mStart;
		Common::Vector2 mEnd;
		float mWidth;
		float mWindowPos;
		float mDoorPos;
};


class House {
	public:
		struct DoorInfo {
			unsigned int WallIndex;
			float DoorPosition;
			float DoorWidth;
		};


		House() { }
		House(const Common::Vector3& p1,
			const Common::Vector3& p2,
			const Common::Vector3& p3,
			const Common::Vector3& p4,
			const std::vector<DoorInfo>& doors);
		bool isInside(float x, float y) const;
		float distanceTo(float x, float y) const;
		const std::vector<HouseWall>& getWalls() const;
		float getHouseFloorHeight() const;
		float getHouseRoofHeight() const;
		Common::Vector3 getCornerstonePosition() const;
		std::pair<std::vector<Common::Vector3>, std::vector<Common::Vector3>> getVertexCoordsAndNormals() const;
		std::vector<Common::Vector2> getTexCoords() const;
		std::vector<unsigned int> getIndices() const;

	private:
		// TODO: allocate house walls from a contiguous array
		std::vector<HouseWall> mWalls;
		std::array<Common::Vector2, 4> mp;
		float mFloor;
		float mRoof;
};

#endif

