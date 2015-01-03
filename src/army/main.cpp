#include <iostream>
#include <sstream>
#include <memory>
#include <vector>
#include <map>
#include <random>

#include <string.h>
#include <assert.h>
#include <float.h>
#include <time.h>

#include <SDL/SDL_mixer.h>

#include <noise/noise.h>

#include <sscene/Scene.h>

#include <common/Clock.h>
#include <common/Math.h>
#include <common/Random.h>
#include <common/DriverFramework.h>

class Sound {
	public:
		static void init();
		static void play(const Common::Vector3& pos);
		static void setCamera(const Common::Vector3& pos, const Common::Vector3& lookdir);

	private:
		static Mix_Chunk* mRifle;
		static unsigned int mNextChannel;
		static Common::Vector3 mCamPos;
		static Common::Vector3 mCamLook;
};

Mix_Chunk* Sound::mRifle = nullptr;
unsigned int Sound::mNextChannel = 0;
Common::Vector3 Sound::mCamPos;
Common::Vector3 Sound::mCamLook;

void Sound::init()
{
	int flags=MIX_INIT_MP3;
	int initted=Mix_Init(flags);
	if((initted & flags) != flags) {
		printf("Mix_Init: Failed to init required ogg and mod support!\n");
		printf("Mix_Init: %s\n", Mix_GetError());
		return;
	}

	// open 44.1KHz, signed 16bit, system byte order,
	//      stereo audio, using 1024 byte chunks
	if(Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024)==-1) {
		printf("Mix_OpenAudio: %s\n", Mix_GetError());
		return;
	}

	Mix_AllocateChannels(16);
	mRifle = Mix_LoadWAV("share/rifle.wav");
	if(!mRifle) {
		printf("Couldn't load rifle sound!\n");
	}
}

void Sound::play(const Common::Vector3& pos)
{
	if(!mRifle)
		return;

	unsigned int dist = static_cast<unsigned int>(mCamPos.distance(pos));
	if(dist > 255)
		return;

	Common::Vector2 om(mCamLook.x, mCamLook.z);
	if(om.null())
		om.x = 1.0f;
	Common::Vector2 oq(pos.x - mCamPos.x, pos.z - mCamPos.z);
	unsigned int angle = 360 + Common::Math::radiansToDegrees(om.angleTo360(oq));
	if(angle > 360)
		angle -= 360;

	Mix_SetPosition(mNextChannel, angle, static_cast<unsigned char>(dist));
	if(Mix_PlayChannel(mNextChannel, mRifle, 0) == -1) {
		printf("Mix_PlayMusic: %s\n", Mix_GetError());
	}
	mNextChannel = (mNextChannel + 1) % 16;
}

void Sound::setCamera(const Common::Vector3& pos, const Common::Vector3& lookdir)
{
	mCamPos = pos;
	mCamLook = lookdir;
}

class WindowFocus {
	public:
		static void setWindowFocus(bool focus);
		static bool getWindowFocus();

	private:
		static bool mWindowFocus;
};

bool WindowFocus::mWindowFocus = false;

void WindowFocus::setWindowFocus(bool focus)
{
	if(focus) {
		SDL_WM_GrabInput(SDL_GRAB_ON);
		SDL_ShowCursor(SDL_DISABLE);
	} else {
		SDL_WM_GrabInput(SDL_GRAB_OFF);
		SDL_ShowCursor(SDL_ENABLE);
	}
	mWindowFocus = focus;
}

bool WindowFocus::getWindowFocus()
{
	return mWindowFocus;
}

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

struct Tree {
	Tree(const Common::Vector3& pos, float radius);
	Common::Vector3 Position;
	float Radius;
	static constexpr float UNPASSABLE_COEFFICIENT = 0.8f;
	static constexpr float TRUNK_COEFFICIENT      = 0.1f;
	static constexpr float HEIGHT_COEFFICIENT     = 2.0f;
};

Tree::Tree(const Common::Vector3& pos, float radius)
	: Position(pos),
	Radius(radius)
{
}

struct Ray {
	Common::Vector3 start;
	Common::Vector3 end;
};

class HouseWall {
	public:
		HouseWall(const Common::Vector2& start, const Common::Vector2& end, float halfwidth);
		const Common::Vector2& getStart() const;
		const Common::Vector2& getEnd() const;
		float getWallHalfWidth() const;

	private:
		Common::Vector2 mStart;
		Common::Vector2 mEnd;
		float mWidth;
};

HouseWall::HouseWall(const Common::Vector2& start, const Common::Vector2& end, float width)
	: mStart(start),
	mEnd(end),
	mWidth(width)
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


class House {
	public:
		House() { }
		House(const Common::Vector3& p1,
			const Common::Vector3& p2,
			const Common::Vector3& p3,
			const Common::Vector3& p4);
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

House::House(const Common::Vector3& p1,
		const Common::Vector3& p2,
		const Common::Vector3& p3,
		const Common::Vector3& p4)
{
	const float halfwidth = 0.25f;
	mp[0] = Common::Vector2(p1.x, p1.z);
	mp[1] = Common::Vector2(p2.x, p2.z);
	mp[2] = Common::Vector2(p3.x, p3.z);
	mp[3] = Common::Vector2(p4.x, p4.z);

	// ensure the walls are attached together on the corners
	auto ext0 = (mp[1] - mp[0]).normalized() * halfwidth;
	auto ext1 = (mp[2] - mp[0]).normalized() * halfwidth;
	auto ext2 = (mp[1] - mp[3]).normalized() * halfwidth;
	auto ext3 = (mp[3] - mp[2]).normalized() * halfwidth;

	mFloor = p1.y;
	mRoof = mFloor + (rand() % 20 + 20) * 0.1f;

	mWalls.push_back(HouseWall(mp[0] - ext0, mp[1] + ext0, halfwidth));
	mWalls.push_back(HouseWall(mp[2] + ext1, mp[0] - ext1, halfwidth));
	mWalls.push_back(HouseWall(mp[1] + ext2, mp[3] - ext2, halfwidth));
	mWalls.push_back(HouseWall(mp[3] + ext3, mp[2] - ext3, halfwidth));
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

class WorldMap {
	public:
		WorldMap(Scene::Scene& scene);
		void create();
		float getHeightAt(float x, float y) const;
		std::vector<Tree> getTreesAt(float x, float y, float r) const;
		Common::Vector3 getNormalAt(float x, float y) const;
		bool lineBlockedByLand(const Common::Vector3& p1, const Common::Vector3& p2, Common::Vector3* hit) const;
		float getWidth() const;
		Common::Vector3 findFreeSpot(unsigned int tries, std::default_random_engine& gen) const;
		const std::vector<House>& getHousesAt(float x, float y, float r) const;
		Common::Vector3 pointToVec(float x, float y) const;

	private:
		void addHouses();
		void addTrees();
		Common::Vector3 findFreeSpot(unsigned int tries);
		bool nearHouse(float x, float y, float radius, House* house) const;
		bool isFreeSpot(float x, float y) const;

		Scene::Scene& mScene;
		std::vector<Tree> mTrees;

		std::default_random_engine mGen;
		noise::module::Perlin mNoise;

		std::vector<House> mHouses;
};

WorldMap::WorldMap(Scene::Scene& scene)
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

Common::Vector3 WorldMap::findFreeSpot(unsigned int tries, std::default_random_engine& gen) const
{
	std::uniform_real_distribution<double> dis(0.0, getWidth());

	for(unsigned int i = 0; i < tries; i++) {
		float x = dis(gen);
		float y = dis(gen);
		if(isFreeSpot(x, y)) {
			return pointToVec(x, y);
		}
	}
	return Common::Vector3();
}

Common::Vector3 WorldMap::findFreeSpot(unsigned int tries)
{
	return findFreeSpot(tries, mGen);
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
	addHouses();
	addTrees();

	auto hmapconv = getWidth() / 128.0f;
	Heightmap hm(128, hmapconv, [&] (float x, float y) { return getHeightAt(x, y); });
	mScene.addModelFromHeightmap("Terrain", hm);
	auto mi = mScene.addMeshInstance("Terrain", "Terrain", "Snow");

	mScene.addPlane("Sea", getWidth(), getWidth(), 16);
	auto mi2 = mScene.addMeshInstance("Sea", "Sea", "Sea");
	mi2->setPosition(Common::Vector3(-getWidth() / 2.0f, 0.0f, -getWidth() / 2.0f));
	mi2->setScale(getWidth() * 2.0f, 1.0f, getWidth() * 2.0f);
}

Common::Vector3 WorldMap::pointToVec(float x, float y) const
{
	return Common::Vector3(x, getHeightAt(x, y), y);
}


void WorldMap::addHouses()
{
	for(int i = 0; i < 50; i++) {
		Common::Vector3 cornerstone = findFreeSpot(100);
		if(cornerstone.null())
			break;

		std::uniform_real_distribution<double> wallLengthDis(5.0f, 15.0f);
		auto corner2x = wallLengthDis(mGen);
		auto corner2z = wallLengthDis(mGen);
		auto corner2 = pointToVec(cornerstone.x + corner2x,
				cornerstone.z + corner2z);

		if(!isFreeSpot(corner2.x, corner2.z)) {
			continue;
		}

		auto corner3_2d = Common::Math::rotate2D(Common::Vector2(corner2x,
					corner2z), HALF_PI);

		auto corner3x = wallLengthDis(mGen);

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
				corner4);
		mHouses.push_back(house);
	}

	for(unsigned int i = 0; i < mHouses.size(); i++) {
		std::stringstream ss;
		ss << "House " << i << "\n";
		std::string hname = ss.str();
		auto vpair = mHouses[i].getVertexCoordsAndNormals();
		auto texcoords = mHouses[i].getTexCoords();
		auto indices = mHouses[i].getIndices();
		mScene.addModel(hname, vpair.first, texcoords, indices, vpair.second);
		auto mi = mScene.addMeshInstance(hname, hname, "House");
		auto cp = mHouses[i].getCornerstonePosition();
		std::cout << "Cornerstone at " << cp << "\n";
		mi->setPosition(cp);
	}
}

void WorldMap::addTrees()
{
	std::uniform_real_distribution<double> radiusdis(2.0f, 5.0f);
	std::uniform_real_distribution<double> rotatdis(0.0f, QUARTER_PI);

	// TODO: Create a single mesh instance with all models instead
	for(int i = 0; i < 500; i++) {
		auto pos = findFreeSpot(1);
		if(!pos.null()) {
			float r = radiusdis(mGen);
			float rot = rotatdis(mGen);
			mTrees.push_back(Tree(pos, r));

			char name[256];
			snprintf(name, 255, "Tree%d", i);

			auto treeInst = mScene.addMeshInstance(name, "Tree", "Tree", false, true);
			treeInst->setPosition(pos);
			treeInst->setScale(r, r, r);
			treeInst->setRotationFromEuler(Common::Vector3(0.0f, rot, 0.0f));
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

class Weapon {
	public:
		Weapon();
		void update(float dt);
		bool canShoot() const;
		void shoot();

	private:
		Common::Countdown mTimer;
		unsigned int mBullets;
};

Weapon::Weapon()
	: mTimer(2.0f),
	mBullets(100)
{
}

void Weapon::update(float dt)
{
	mTimer.doCountdown(dt);
	mTimer.check();
}

bool Weapon::canShoot() const
{
	return !mTimer.running() && mBullets;
}

void Weapon::shoot()
{
	assert(canShoot());
	mTimer.rewind();
	if(mBullets > 0)
		mBullets--;
}

template <typename Physics>
class PhysicsCommon {
	public:
		static void update(float dt, Physics& p);
};

template <typename Physics>
void PhysicsCommon<Physics>::update(float dt, Physics& p)
{
	p.mAcceleration.y -= 10.0f;
	p.mVelocity += p.mAcceleration * dt;
	if(p.mMaxVel) {
		p.mVelocity.truncate(p.mMaxVel);
	}

	if(dt && p.mVelFriction) {
		p.mAcceleration += p.mVelocity.normalized() * (1.0f - p.mVelFriction);
	}

	auto oldpos = p.mPosition;
	p.mPosition += p.mVelocity * dt;
	p.checkTreeCollision(oldpos);
	p.checkLandCollision(oldpos);
	p.checkWallCollision(oldpos);

	p.mAcceleration = Common::Vector3();
}

class SoldierPhysics {
	public:
		SoldierPhysics() = default;
		SoldierPhysics(const WorldMap* wmap, float maxvel, float velfriction, float friction);
		void update(float dt);
		void addAcceleration(const Common::Vector3& vec);
		const Common::Vector3& getPosition() const { return mPosition; }
		const Common::Vector3& getVelocity() const { return mVelocity; }
		const Common::Quaternion& getOrientation() const { return mOrientation; }
		Common::Quaternion getAimPitch() const;
		void rotate(float yaw, float pitch);

		void setPosition(const Common::Vector3& pos) { mPosition = pos; }
		void setOrientation(const Common::Quaternion& ori) { mOrientation = ori; }
		void setVelocity(const Common::Vector3& v) { mVelocity = v; }

		void setAiming(bool a);
		bool isAiming() const;

	private:
		void checkTreeCollision(const Common::Vector3& oldpos);
		void checkLandCollision(const Common::Vector3& oldpos);
		void checkWallCollision(const Common::Vector3& oldpos);

		Common::Vector3 mPosition;
		Common::Vector3 mVelocity;
		Common::Vector3 mAcceleration;
		Common::Quaternion mOrientation;
		float mAimPitch;
		const WorldMap* mMap;
		float mMaxVel;
		float mVelFriction;
		float mFriction;
		bool mAiming;

		friend class PhysicsCommon<SoldierPhysics>;
};

SoldierPhysics::SoldierPhysics(const WorldMap* wmap, float maxvel, float velfriction, float friction)
	: mAimPitch(0.0f),
	mMap(wmap),
	mMaxVel(maxvel),
	mVelFriction(velfriction),
	mFriction(friction),
	mAiming(false)
{
}

void SoldierPhysics::update(float dt)
{
	PhysicsCommon<SoldierPhysics>::update(dt, *this);
}

void SoldierPhysics::checkLandCollision(const Common::Vector3& oldpos)
{
	float hgt = mMap->getHeightAt(mPosition.x, mPosition.z);
	float ydiff = hgt - mPosition.y;
	if(ydiff > 0.0f) {
		mPosition.y += ydiff;
		mVelocity = mVelocity * mFriction;
	}

	if(hgt < -1.2f) {
		auto oldhgt = mMap->getHeightAt(oldpos.x, oldpos.z);
		if(oldhgt > hgt) {
			mPosition = oldpos;
		}
	}
}

void SoldierPhysics::checkTreeCollision(const Common::Vector3& oldpos)
{
	for(const auto& t : mMap->getTreesAt(mPosition.x, mPosition.z, 5.0f)) {
		auto rad = t.Radius * Tree::UNPASSABLE_COEFFICIENT;
		auto rad2 = rad * rad;
		if(t.Position.distance2(mPosition) < rad2) {
			auto diff = (mPosition - t.Position).normalized();
			auto newpos = t.Position + diff * rad;
			mPosition = newpos;
		}
	}
}

void SoldierPhysics::checkWallCollision(const Common::Vector3& oldpos)
{
	auto travdist = oldpos.distance(mPosition);
	for(const auto& house : mMap->getHousesAt(mPosition.x, mPosition.z, 5.0f)) {
		for(const auto& wall : house.getWalls()) {
			const auto& rs = wall.getStart();
			const auto& re = wall.getEnd();
			float width = wall.getWallHalfWidth();
			Common::Vector2 mp = Common::Vector2(mPosition.x, mPosition.z);
			Common::Vector2 op = Common::Vector2(oldpos.x, oldpos.z);

			Common::Vector2 nearest;

			auto dist = Common::Math::pointToSegmentDistance(
					rs,
					re,
					mp, &nearest);
			if(dist < width + 0.5f) {
				// soldier is within wall
				std::cout << "Soldier is within wall\n";
				auto vecFromWall = op +
					(op - nearest).normalized() * (width + 0.5f);
				mPosition = mMap->pointToVec(vecFromWall.x, vecFromWall.y);
			} else if(travdist > width) {
				// not within wall but possibly tunneled
				bool found;
				auto v = Common::Math::segmentSegmentIntersection2D(
						rs,
						re,
						op,
						mp,
						&found);
				if(found) {
					std::cout << "Soldier tunneled through wall " << rs << "\t" << re << "\n";
					std::cout << "Dist: " << dist << "\toldpos: " << op << "\t" << mp << "\n";
					std::cout << "v: " << v << "\n";
					mPosition = oldpos;
				}
			}
		}
	}
}

void SoldierPhysics::addAcceleration(const Common::Vector3& vec)
{
	if(mAiming)
		return;

	mAcceleration += vec;
	mAcceleration.truncate(1.0f);
}

Common::Quaternion SoldierPhysics::getAimPitch() const
{
	return Common::Quaternion::fromAxisAngle(Common::Vector3(0.0f, 0.0f, 1.0f), mAimPitch);
}

void SoldierPhysics::rotate(float yaw, float pitch)
{
	mOrientation = mOrientation *
		Common::Quaternion::fromAxisAngle(Common::Vector3(0.0f, 1.0f, 0.0f), yaw);
	mAimPitch = Common::clamp<float>(-HALF_PI * 0.6f, mAimPitch + pitch, HALF_PI * 0.6f);
}

void SoldierPhysics::setAiming(bool a)
{
	mAiming = a;
}

bool SoldierPhysics::isAiming() const
{
	return mAiming;
}

class BulletPhysics {
	public:
		BulletPhysics() = default;
		BulletPhysics(const WorldMap* wmap, float maxvel, float velfriction, float friction);
		void update(float dt);
		const Common::Vector3& getPosition() const { return mPosition; }
		const Common::Vector3& getVelocity() const { return mVelocity; }

		void setPosition(const Common::Vector3& pos) { mPosition = pos; }
		void setVelocity(const Common::Vector3& v) { mVelocity = v; }
		bool isActive() const { return mActive; }

	private:
		void checkTreeCollision(const Common::Vector3& oldpos);
		void checkLandCollision(const Common::Vector3& oldpos);
		void checkWallCollision(const Common::Vector3& oldpos);

		Common::Vector3 mPosition;
		Common::Vector3 mVelocity;
		Common::Vector3 mAcceleration;
		const WorldMap* mMap;
		float mMaxVel;
		float mVelFriction;
		float mFriction;
		bool mActive;

		static constexpr float BULLET_SLOWDOWN_PER_TREE_METER = 1000.0f;
		friend class PhysicsCommon<BulletPhysics>;
};

BulletPhysics::BulletPhysics(const WorldMap* wmap, float maxvel, float velfriction, float friction)
	: mMap(wmap),
	mMaxVel(maxvel),
	mVelFriction(velfriction),
	mFriction(friction),
	mActive(true)
{
}

void BulletPhysics::update(float dt)
{
	if(mVelocity.length() < 1.0f) {
		mActive = false;
	}

	if(!mActive)
		return;

	PhysicsCommon<BulletPhysics>::update(dt, *this);
}

void BulletPhysics::checkLandCollision(const Common::Vector3& oldpos)
{
	Common::Vector3 hitpoint;
	if(mMap->lineBlockedByLand(oldpos, mPosition, &hitpoint)) {
		mPosition = hitpoint;
		mVelocity.zero();
	}
}

void BulletPhysics::checkWallCollision(const Common::Vector3& oldpos)
{
	for(const auto& house : mMap->getHousesAt(mPosition.x, mPosition.z, oldpos.distance(mPosition) + 5.0f)) {
		for(const auto& wall : house.getWalls()) {
			const auto& rs = wall.getStart();
			const auto& re = wall.getEnd();
			Common::Vector2 mp = Common::Vector2(mPosition.x, mPosition.z);
			Common::Vector2 op = Common::Vector2(oldpos.x, oldpos.z);

			bool found;
			auto hitpoint = Common::Math::segmentSegmentIntersection2D(
					rs,
					re,
					op,
					mp,
					&found);
			if(found) {
				mPosition = mMap->pointToVec(hitpoint.x, hitpoint.y);
				mVelocity.zero();
				break;
			}
		}
	}
}

void BulletPhysics::checkTreeCollision(const Common::Vector3& oldpos)
{
	for(const auto& t : mMap->getTreesAt(mPosition.x, mPosition.z, 5.0f)) {
		auto treeHeight = t.Position.y + t.Radius * Tree::HEIGHT_COEFFICIENT;
		if(oldpos.y > treeHeight && mPosition.y > treeHeight)
			return;

		Common::Vector2 nearest;
		Common::Vector2 oldpos2(oldpos.x, oldpos.z);
		Common::Vector2 pos2(mPosition.x, mPosition.z);
		auto dist = Common::Math::pointToSegmentDistance(oldpos2,
				pos2,
				Common::Vector2(t.Position.x, t.Position.z), &nearest);
		auto rad = t.Radius * Tree::TRUNK_COEFFICIENT;

		if(dist < rad) {
			auto distTravelledInTrunk = rad - dist;
			auto currSpeed = mVelocity.length();
			auto speed = std::max<double>(0.0,
					currSpeed - distTravelledInTrunk * BULLET_SLOWDOWN_PER_TREE_METER);
			mVelocity = mVelocity.normalized() * speed;

			if(mVelocity.length() < 1.0f) {
				auto distFromStartToImpact = oldpos2.distance(nearest);
				auto distFromEndToImpact = pos2.distance(nearest);
				auto distCoeff = distFromStartToImpact / (distFromStartToImpact + distFromEndToImpact);
				auto newHeight = oldpos.y + distCoeff * (mPosition.y - oldpos.y);
				mPosition = Common::Vector3(nearest.x, newHeight, nearest.y);
			}
		}
	}
}


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

class BulletRenderer {
	public:
		BulletRenderer();
		BulletRenderer(Scene::Scene* scene, const HitterComponent* hit, unsigned int id);
		void update(float dt);

	private:
		Scene::Scene* mScene;
		const HitterComponent* mHitter;
		unsigned int mID;
		std::string mName;
		unsigned int mLineTicks;
};

BulletRenderer::BulletRenderer()
{
}

BulletRenderer::BulletRenderer(Scene::Scene* scene, const HitterComponent* hit, unsigned int id)
	: mScene(scene),
	mHitter(hit),
	mLineTicks(0)
{
	std::stringstream ss;
	ss << "Bullet line " << id;
	mName = ss.str();
}

void BulletRenderer::update(float dt)
{
	auto r = mHitter->getLastRay();
	if(mHitter->isActive() || (mLineTicks && r.start != r.end)) {
		mScene->addLine(mName, r.start, r.end, Common::Color::Red);
		mLineTicks++;
	}

	if(mLineTicks > 5) {
		mLineTicks = 0;
		mScene->clearLine(mName);
	}
}

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
	velvec.x += Common::Random::clamped() * 0.01f;
	velvec.y += Common::Random::clamped() * 0.01f;
	velvec.z += Common::Random::clamped() * 0.01f;
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

class RenderComponent {
	public:
		RenderComponent(Scene::Scene& scene,
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

class Soldiers {
	public:
		Soldiers(Scene::Scene& scene);
		void update(float dt);
		void addSoldiers(const WorldMap* wmap, Bullets* bullets);
		const Common::Vector3& getPlayerSoldierPosition() const;
		Common::Quaternion getPlayerSoldierOrientation() const;
		bool getPlayerSoldierAiming() const;
		SoldierPhysics* getPlayerPhysics();
		ShooterComponent* getPlayerShooter();
		HittableComponent* getPlayerHittable();
		SoldierPhysics* getPhys(unsigned int i);
		const SoldierPhysics* getPhys(unsigned int i) const;
		const Common::Vector3& getSoldierPosition(unsigned int id) const;
		ShooterComponent* getShooter(unsigned int i);
		HittableComponent* getHittable(unsigned int i);
		std::vector<HittableComponent> getHittables() const;
		void processHits(const std::vector<unsigned int>& hits);
		unsigned int getNumSoldiers() const;
		unsigned int getPlayerSoldierIndex() const;
		std::vector<unsigned int> getSoldiersAt(const Common::Vector3& pos, float radius) const;
		bool soldierIsAlive(unsigned int i) const;

		static const unsigned int MAX_SOLDIERS = 256;

	private:
		unsigned int mNumSoldiers;
		std::vector<SoldierPhysics> mPhysics;
		std::vector<RenderComponent> mRenders;
		std::vector<ShooterComponent> mShooters;
		std::vector<HittableComponent> mHittables;

		Scene::Scene& mScene;
		unsigned int mPlayerSoldierIndex;
};

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

void Soldiers::addSoldiers(const WorldMap* wmap, Bullets* bullets)
{
	const unsigned int numSoldiers = 8;

	mPlayerSoldierIndex = 0;
	std::default_random_engine gen((unsigned int)time(0));
	for(unsigned int i = 0; i < numSoldiers; i++) {
		mPhysics[i] = SoldierPhysics(wmap, 5.0f, 0.95f, 1.0f);
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

class PlayerInput {
	public:
		PlayerInput() = default;
		PlayerInput(Soldiers* soldiers);
		void update(float dt);
		bool handleKeyDown(float frameTime, SDLKey key);
		bool handleKeyUp(float frameTime, SDLKey key);
		bool handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev);
		bool handleMousePress(float frameTime, Uint8 button);
		void changeFOV(float v);
		float getFOV() const;

	private:
		SoldierPhysics* mPhys;
		ShooterComponent* mShooter;
		HittableComponent* mHittable;
		Common::Vector3 mInputAccel;
		float mFOV;
};

PlayerInput::PlayerInput(Soldiers* soldiers)
	: mPhys(soldiers->getPlayerPhysics()),
	mShooter(soldiers->getPlayerShooter()),
	mHittable(soldiers->getPlayerHittable()),
	mFOV(90.0f)
{
}

void PlayerInput::update(float dt)
{
	if(mHittable->hasDied())
		return;

	Common::Vector3 accel;
	accel = Common::Math::rotate3D(mInputAccel, mPhys->getOrientation());
	mPhys->addAcceleration(accel);
}

bool PlayerInput::handleKeyDown(float frameTime, SDLKey key)
{
	switch(key) {
		case SDLK_ESCAPE:
			return true;

		case SDLK_w:
			mInputAccel.x = 1.0f;
			break;

		case SDLK_q:
			mInputAccel.y = 1.0f;
			break;

		case SDLK_d:
			mInputAccel.z = 1.0f;
			break;

		case SDLK_s:
			mInputAccel.x = -1.0f;
			break;

		case SDLK_e:
			mInputAccel.y = -1.0f;
			break;

		case SDLK_a:
			mInputAccel.z = -1.0f;
			break;

		case SDLK_p:
			{
				std::cout << "Position: " << mPhys->getPosition() << "\n";
				std::cout << "Velocity: " << mPhys->getVelocity() << "\n";
				std::cout << "Orientation: " << mPhys->getOrientation() << "\n";
				std::cout << "Aim pitch: " << mPhys->getAimPitch() << "\n";
			}
			break;

		default:
			break;
	}

	return false;
}

bool PlayerInput::handleKeyUp(float frameTime, SDLKey key)
{
	switch(key) {
		case SDLK_w:
		case SDLK_s:
			mInputAccel.x = 0.0f;
			break;

		case SDLK_q:
		case SDLK_e:
			mInputAccel.y = 0.0f;
			break;

		case SDLK_d:
		case SDLK_a:
			mInputAccel.z = 0.0f;
			break;

		default:
			break;
	}
	return false;
}

bool PlayerInput::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	if(mHittable->hasDied())
		return false;

	const float coeff = mPhys->isAiming() ? 0.005f : 0.02f;

	mPhys->rotate(-ev.xrel * coeff, -ev.yrel * coeff);
	return false;
}

bool PlayerInput::handleMousePress(float frameTime, Uint8 button)
{
	if(mHittable->hasDied())
		return false;

	if(button == SDL_BUTTON_LEFT) {
		mShooter->shoot();
	} else if(button == SDL_BUTTON_RIGHT) {
		auto a = mPhys->isAiming();
		mPhys->setAiming(!a);
	} else if(button == SDL_BUTTON_WHEELUP) {
		changeFOV(-5);
	} else if(button == SDL_BUTTON_WHEELDOWN) {
		changeFOV(5);
	}
	return false;
}

void PlayerInput::changeFOV(float v)
{
	mFOV = Common::clamp<float>(90.0f, mFOV + v, 120.0f);
}

float PlayerInput::getFOV() const
{
	return mFOV;
}

class SoldierKnowledge {
	public:
		SoldierKnowledge(unsigned int id, const Common::Vector3& pos);
		bool isCurrentlySeen() const { return mCurrentlySeen; }
		const Common::Vector3& getPosition() const { return mLastKnownPosition; }
		unsigned int getID() const { return mID; }

	private:
		Common::Vector3 mLastKnownPosition;
		bool mCurrentlySeen;
		unsigned int mID;
};

SoldierKnowledge::SoldierKnowledge(unsigned int id, const Common::Vector3& pos)
	: mLastKnownPosition(pos),
	mCurrentlySeen(true),
	mID(id)
{
}

class AISensor {
	public:
		AISensor() = default;
		AISensor(const WorldMap* wmap, const Soldiers* soldiers, unsigned int id);
		void update(float dt);
		std::vector<SoldierKnowledge> getCurrentlySeenEnemies() const;
		bool canSee(unsigned int id) const;

	private:
		const WorldMap* mMap;
		const Soldiers* mSoldiers;
		const SoldierPhysics* mPhys;
		std::vector<SoldierKnowledge> mSensedSoldiers;
		unsigned int mID;
};

AISensor::AISensor(const WorldMap* wmap, const Soldiers* soldiers, unsigned int id)
	: mMap(wmap),
	mSoldiers(soldiers),
	mPhys(soldiers->getPhys(id)),
	mID(id)
{
}

void AISensor::update(float dt)
{
	mSensedSoldiers.clear();
	auto& mypos = mPhys->getPosition();
	auto mydir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation());
	for(auto& s : mSoldiers->getSoldiersAt(mypos, 100.0f)) {
		if(s != mID && canSee(s) && mSoldiers->soldierIsAlive(s)) {
			auto& othpos = mSoldiers->getSoldierPosition(s);
			Common::Vector3 posdiff = othpos - mypos;
			auto ang = Common::Vector2(posdiff.x, posdiff.z).angleTo(Common::Vector2(mydir.x, mydir.z));
			if(ang < QUARTER_PI) {
				mSensedSoldiers.push_back(SoldierKnowledge(s, othpos));
			}
		}
	}
}

std::vector<SoldierKnowledge> AISensor::getCurrentlySeenEnemies() const
{
	std::vector<SoldierKnowledge> ret;
	for(const auto& s : mSensedSoldiers) {
		if(s.isCurrentlySeen())
			ret.push_back(s);
	}
	return ret;
}

bool AISensor::canSee(unsigned int id) const
{
	// TODO: take trees into account
	return !mMap->lineBlockedByLand(mPhys->getPosition() + Common::Vector3(0.0f, 1.7f, 0.0f),
			mSoldiers->getSoldierPosition(id) + Common::Vector3(0.0f, 1.65f, 0.0f),
			nullptr);
}

class AITask {
	public:
		enum class Type {
			Idle,
			Move,
			Shoot
		};

		Common::Vector3 Vec;
		Type Type;
};

class AIPlanner {
	public:
		AIPlanner() = default;
		AIPlanner(const SoldierPhysics* phys);
		AITask getNextTask(const AISensor& sensor);

	private:
		const SoldierPhysics* mPhys;
};

AIPlanner::AIPlanner(const SoldierPhysics* phys)
	: mPhys(phys)
{
}

AITask AIPlanner::getNextTask(const AISensor& sensor)
{
	auto enemiesSeen = sensor.getCurrentlySeenEnemies();
	if(enemiesSeen.size() > 0) {
		auto mypos = mPhys->getPosition();
		std::sort(enemiesSeen.begin(), enemiesSeen.end(), [&](const SoldierKnowledge& a, const SoldierKnowledge& b) {
				return mypos.distance2(a.getPosition()) <
				mypos.distance2(b.getPosition()); });
		AITask t;
		t.Type = AITask::Type::Shoot;
		t.Vec = enemiesSeen[0].getPosition();
		return t;
	}

	AITask t;
	t.Type = AITask::Type::Move;
	t.Vec = Common::Vector3(64.0f, 0.0f, 64.0f);
	return t;
}

class AIActor {
	public:
		AIActor() = default;
		AIActor(SoldierPhysics* phys, ShooterComponent* shooter);
		void execute(const AITask& t);

	private:
		SoldierPhysics* mPhys;
		ShooterComponent* mShooter;
};

AIActor::AIActor(SoldierPhysics* phys, ShooterComponent* shooter)
	: mPhys(phys),
	mShooter(shooter)
{
}

void AIActor::execute(const AITask& t)
{
	switch(t.Type) {
		case AITask::Type::Idle:
			break;

		case AITask::Type::Move:
			{
				Common::Vector3 accel;
				accel = t.Vec - mPhys->getPosition();
				accel.y = 0.0f;
				if(accel.length() > 1.0f) {
					mPhys->addAcceleration(accel);
				}
			}
			break;

		case AITask::Type::Shoot:
			{
				auto currdir = Common::Math::rotate3D(Scene::WorldForward, mPhys->getOrientation() * mPhys->getAimPitch());
				auto tgtvec = t.Vec - mPhys->getPosition();
				auto curryaw = atan2(currdir.z, currdir.x);
				auto tgtyaw = atan2(tgtvec.z, tgtvec.x) + Common::Random::clamped() * 0.2f;
				auto currpitch = atan2(currdir.y, 1.0f);
				auto tgtpitch = atan2(tgtvec.normalized().y, 1.0f) + Common::Random::clamped() * 0.2f;
				auto diffyaw = curryaw - tgtyaw;
				auto diffpitch = -(currpitch - tgtpitch);
				mPhys->rotate(0.02f * diffyaw, 0.02f * diffpitch);
				if(fabs(diffyaw) < 0.2f && fabs(diffpitch) < 0.2f) {
					mShooter->shoot();
				}
				mPhys->addAcceleration(currdir);
			}
			break;
	}
}

class AIComponent {
	public:
		AIComponent() = default;
		AIComponent(const WorldMap* wmap, Soldiers* soldiers, unsigned int id);
		void update(float dt);

	private:
		const Soldiers* mSoldiers;
		AISensor mSensor;
		AIPlanner mPlanner;
		AIActor mActor;
		SoldierPhysics* mPhys;
		ShooterComponent* mShooter;
		HittableComponent* mHittable;
};

AIComponent::AIComponent(const WorldMap* wmap, Soldiers* soldiers, unsigned int id)
	: mSoldiers(soldiers),
	mSensor(wmap, soldiers, id),
	mPlanner(soldiers->getPhys(id)),
	mActor(soldiers->getPhys(id), soldiers->getShooter(id)),
	mPhys(soldiers->getPhys(id)),
	mShooter(soldiers->getShooter(id)),
	mHittable(soldiers->getHittable(id))
{
}

void AIComponent::update(float dt)
{
	if(mHittable->hasDied())
		return;

	mSensor.update(dt);
	auto task = mPlanner.getNextTask(mSensor);
	mActor.execute(task);
}

class AI {
	public:
		AI() = default;
		AI(const WorldMap* wmap, Soldiers* soldiers, const Bullets* bullets);
		void init();
		void update(float dt);

	private:
		const WorldMap* mMap;
		Soldiers* mSoldiers;
		const Bullets* mBullets;
		std::vector<AIComponent> mAIs;
		unsigned int mNumAIs;
};

AI::AI(const WorldMap* wmap, Soldiers* soldiers, const Bullets* bullets)
	: mMap(wmap),
	mSoldiers(soldiers),
	mBullets(bullets)
{
	mAIs.resize(Soldiers::MAX_SOLDIERS);
}

void AI::init()
{
	mNumAIs = mSoldiers->getNumSoldiers();
	for(unsigned int i = 0; i < mNumAIs; i++) {
		mAIs[i] = AIComponent(mMap, mSoldiers, i);
	}
}

void AI::update(float dt)
{
	auto plid = mSoldiers->getPlayerSoldierIndex();
	for(unsigned int i = 0; i < mNumAIs; i++) {
		if(i != plid) {
			mAIs[i].update(dt);
		}
	}
}

class World {
	public:
		World(Scene::Scene& scene);
		void init();
		void update(float dt);
		bool handleKeyDown(float frameTime, SDLKey key);
		bool handleKeyUp(float frameTime, SDLKey key);
		bool handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev);
		bool handleMousePress(float frameTime, Uint8 button);

	private:
		WorldMap mMap;
		Soldiers mSoldiers;
		Bullets mBullets;
		AI mAI;
		Scene::Scene& mScene;
		bool mObserverMode;
		Scene::Camera& mCamera;
		PlayerInput mPlayerInput;
		std::map<SDLKey, std::function<void (float)>> mControls;
};

World::World(Scene::Scene& scene)
	: mMap(scene),
	mSoldiers(scene),
	mBullets(&mMap, &scene),
	mScene(scene),
	mObserverMode(false),
	mCamera(mScene.getDefaultCamera())
{
	mControls[SDLK_w] = [&] (float p) { mCamera.setForwardMovement(p); };
	mControls[SDLK_q] = [&] (float p) { mCamera.setUpwardsMovement(p); };
	mControls[SDLK_d] = [&] (float p) { mCamera.setSidewaysMovement(p); };
	mControls[SDLK_s] = [&] (float p) { mCamera.setForwardMovement(-p); };
	mControls[SDLK_e] = [&] (float p) { mCamera.setUpwardsMovement(-p); };
	mControls[SDLK_a] = [&] (float p) { mCamera.setSidewaysMovement(-p); };
}

void World::init()
{
	mMap.create();
	mSoldiers.addSoldiers(&mMap, &mBullets);
	mPlayerInput = PlayerInput(&mSoldiers);
	mAI = AI(&mMap, &mSoldiers, &mBullets);
	mAI.init();
}

void World::update(float dt)
{
	if(mObserverMode) {
		mCamera.applyMovementKeys(dt);
		Sound::setCamera(mCamera.getPosition(), mCamera.getTargetVector());
		mScene.setOverlayEnabled("Sight", false);
	}

	mSoldiers.update(dt);
	mPlayerInput.update(dt);
	mAI.update(dt);
	mBullets.update(dt);

	auto hits = mBullets.checkForHits(mSoldiers.getHittables());
	mSoldiers.processHits(hits);

	if(!mObserverMode) {
		auto ori = mSoldiers.getPlayerSoldierOrientation();
		auto tgt = Common::Math::rotate3D(Scene::WorldForward, ori);
		auto up = Common::Math::rotate3D(Scene::WorldUp, ori);

		mCamera.setPosition(mSoldiers.getPlayerSoldierPosition() +
				Common::Vector3(0.0f, 1.7f, 0.0f) +
				tgt.normalized() * 0.5f);
		mCamera.lookAt(tgt, up);
		Sound::setCamera(mCamera.getPosition(), mCamera.getTargetVector());
		mScene.setOverlayEnabled("Sight", mSoldiers.getPlayerSoldierAiming());
		mScene.setFOV(mPlayerInput.getFOV());
	}
}

bool World::handleKeyDown(float frameTime, SDLKey key)
{
	switch(key) {
		case SDLK_TAB:
			if(SDL_GetModState() & KMOD_ALT) {
				SDL_WM_IconifyWindow();
				return false;
			}
			break;

		case SDLK_RCTRL:
		case SDLK_LCTRL:
		case SDLK_RALT:
		case SDLK_LALT:
			if((SDL_GetModState() & KMOD_CTRL) && (SDL_GetModState() & KMOD_ALT)) {
				WindowFocus::setWindowFocus(false);
				return false;
			}
			break;

		default:
			break;
	}

	if(key == SDLK_F1) {
		mObserverMode = !mObserverMode;
		return false;
	}

	if(mObserverMode) {
		auto it = mControls.find(key);
		if(it != mControls.end()) {
			it->second(0.5f);
			return false;
		} else {
			if(key == SDLK_ESCAPE) {
				return true;
			}
			else if(key == SDLK_p) {
				std::cout << "Position: " << mCamera.getPosition() << "\n";
			} else if(key == SDLK_F7) {
				mScene.setWireframe(true);
			} else if(key == SDLK_F8) {
				mScene.setWireframe(false);
			}
		}
		return false;
	} else {
		return mPlayerInput.handleKeyDown(frameTime, key);
	}
}

bool World::handleKeyUp(float frameTime, SDLKey key)
{
	if(mObserverMode) {
		auto it = mControls.find(key);
		if(it != mControls.end()) {
			it->second(0.0f);
		}
		return false;
	} else {
		return mPlayerInput.handleKeyUp(frameTime, key);
	}
}

bool World::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	if(mObserverMode) {
		mCamera.rotate(-ev.xrel * 0.02f, -ev.yrel * 0.02f);
		return false;
	} else {
		return mPlayerInput.handleMouseMotion(frameTime, ev);
	}
}

bool World::handleMousePress(float frameTime, Uint8 button)
{
	if(button == SDL_BUTTON_LEFT && !WindowFocus::getWindowFocus()) {
		WindowFocus::setWindowFocus(true);
		return false;
	}

	if(mObserverMode) {
		return false;
	} else {
		return mPlayerInput.handleMousePress(frameTime, button);
	}
}

class AppDriver : public Common::Driver {
	public:
		AppDriver();
		virtual void drawFrame() override;
		virtual bool handleKeyDown(float frameTime, SDLKey key) override;
		virtual bool handleKeyUp(float frameTime, SDLKey key) override;
		virtual bool handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev) override;
		virtual bool handleMousePress(float frameTime, Uint8 button) override;
		virtual bool prerenderUpdate(float frameTime) override;

	private:
		Scene::Scene mScene;
		World mWorld;
};

AppDriver::AppDriver()
	: Common::Driver(800, 600, "Army"),
	mScene(800, 600),
	mWorld(mScene)
{
	mScene.addModel("Soldier", "share/soldier.obj");
	mScene.addModel("Tree", "share/tree.obj");
	mScene.addTexture("Snow", "share/snow.jpg");
	mScene.addTexture("Sea", "share/sea.jpg");
	mScene.addTexture("House", "share/house.jpg");
	mScene.addTexture("Soldier", "share/soldier.jpg");
	mScene.addTexture("Tree", "share/tree.png");
	mScene.addOverlay("Sight", "share/sight.png");

	mScene.getAmbientLight().setState(true);
	mScene.getAmbientLight().setColor(Common::Color(127, 127, 127));

	mScene.getDirectionalLight().setState(true);
	mScene.getDirectionalLight().setDirection(Common::Vector3(0.5f, -1.0f, 0.5f));
	mScene.getDirectionalLight().setColor(Common::Vector3(0.9f, 0.9f, 0.9f));

	mScene.setZFar(1024.0f);
	mScene.setClearColor(Common::Color(138, 163, 200));

	mWorld.init();
	Sound::init();
	WindowFocus::setWindowFocus(true);
}

bool AppDriver::handleKeyDown(float frameTime, SDLKey key)
{
	return mWorld.handleKeyDown(frameTime, key);
}

bool AppDriver::handleKeyUp(float frameTime, SDLKey key)
{
	return mWorld.handleKeyUp(frameTime, key);
}

bool AppDriver::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	return mWorld.handleMouseMotion(frameTime, ev);
}

bool AppDriver::handleMousePress(float frameTime, Uint8 button)
{
	return mWorld.handleMousePress(frameTime, button);
}

bool AppDriver::prerenderUpdate(float frameTime)
{
	mWorld.update(frameTime);

	return false;
}

void AppDriver::drawFrame()
{
	mScene.render();
}

class App {
	public:
		void go();

	private:
		AppDriver mDriver;
};

void App::go()
{
	mDriver.run();
}

int main(void)
{
	App a;
	a.go();
	return 0;
}
