#include <iostream>
#include <sstream>
#include <memory>
#include <vector>
#include <map>
#include <random>

#include <string.h>
#include <assert.h>

#include <sscene/Scene.h>

#include <common/Clock.h>
#include <common/Math.h>
#include <common/DriverFramework.h>

class Heightmap : public Scene::Heightmap {
	public:
		Heightmap(std::function<float (float, float)> func);
		virtual float getHeightAt(float x, float y) const;
		virtual float getWidth() const;

	private:
		std::function<float (float, float)> mHeightFunc;
};

Heightmap::Heightmap(std::function<float (float, float)> func)
	: mHeightFunc(func)
{
}

float Heightmap::getHeightAt(float x, float y) const
{
	return mHeightFunc(x, y);
}

float Heightmap::getWidth() const
{
	return 128;
}

class WorldMap {
	public:
		WorldMap(Scene::Scene& scene);
		void create();
		float getHeightAt(float x, float y) const;
		Common::Vector3 getNormalAt(float x, float y) const;

	private:
		Scene::Scene& mScene;
		std::vector<Common::Vector3> mTrees;

		std::default_random_engine mGen;
};

WorldMap::WorldMap(Scene::Scene& scene)
	: mScene(scene)
{
}

void WorldMap::create()
{
	Heightmap hm([&] (float x, float y) { return getHeightAt(x, y); });
	mScene.addModelFromHeightmap("Terrain", hm);
	auto mi = mScene.addMeshInstance("Terrain", "Terrain", "Snow");
	mi->setPosition(Common::Vector3(hm.getWidth() * 0.5f, 0.0f, hm.getWidth() * 0.5f));

	std::uniform_real_distribution<double> treeDis(0.0, hm.getWidth());
	for(int i = 0; i < 500; i++) {
		float x = treeDis(mGen);
		float y = treeDis(mGen);

		auto pos = Common::Vector3(x, getHeightAt(x, y), y);
		mTrees.push_back(pos);

		char name[256];
		snprintf(name, 255, "Tree%d", i);

		auto treeInst = mScene.addMeshInstance(name, "Tree", "Snow");
		treeInst->setPosition(pos);
	}
}

float WorldMap::getHeightAt(float x, float y) const
{
	return 3.0f * sin(x * 0.20f) + 5.0f * cos(y * 0.10f) - 8.0f;
}

Common::Vector3 WorldMap::getNormalAt(float x, float y) const
{
	Common::Vector3 p1(x,
			getHeightAt(x, y),
			y);
	Common::Vector3 p2(x + 0.5f,
			getHeightAt(x + 0.5f, y),
			y);
	Common::Vector3 p3(x,
			getHeightAt(x, y + 0.5f),
			y + 0.5f);
	Common::Vector3 u(p2 - p1);
	Common::Vector3 v(p3 - p1);
	return v.cross(u).normalized();
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
	: mTimer(0.5f),
	mBullets(10)
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

class PhysicsComponent {
	public:
		PhysicsComponent() = default;
		PhysicsComponent(const WorldMap* wmap, float maxvel, float velfriction, float friction);
		void update(float dt);
		void addAcceleration(const Common::Vector3& vec);
		const Common::Vector3& getPosition() const { return mPosition; }
		const Common::Vector3& getVelocity() const { return mVelocity; }
		const Common::Quaternion& getOrientation() const { return mOrientation; }
		const Common::Quaternion& getPitch() const { return mPitch; }
		void rotate(float yaw, float pitch);

		void setPosition(const Common::Vector3& pos) { mPosition = pos; }
		void setOrientation(const Common::Quaternion& ori) { mOrientation = ori; }
		void setVelocity(const Common::Vector3& v) { mVelocity = v; }

	private:
		Common::Vector3 mPosition;
		Common::Vector3 mVelocity;
		Common::Vector3 mAcceleration;
		Common::Quaternion mOrientation;
		Common::Quaternion mPitch;
		const WorldMap* mMap;
		float mMaxVel;
		float mVelFriction;
		float mFriction;
};

PhysicsComponent::PhysicsComponent(const WorldMap* wmap, float maxvel, float velfriction, float friction)
	: mMap(wmap),
	mMaxVel(maxvel),
	mVelFriction(velfriction),
	mFriction(friction)
{
}

void PhysicsComponent::update(float dt)
{
	mAcceleration.y -= 10.0f;
	mVelocity += mAcceleration * dt;
	if(mMaxVel) {
		mVelocity.truncate(mMaxVel);
	}

	if(dt && mVelFriction) {
		mAcceleration += mVelocity.normalized() * (1.0f - mVelFriction);
	}

	mPosition += mVelocity * dt;
	float hgt = mMap->getHeightAt(mPosition.x, mPosition.z);
	float ydiff = hgt - mPosition.y;
	if(ydiff > 0.0f) {
		mPosition.y += ydiff;
		mVelocity = mVelocity * mFriction;
	}

	mAcceleration = Common::Vector3();
}

void PhysicsComponent::addAcceleration(const Common::Vector3& vec)
{
	mAcceleration += vec;
	mAcceleration.truncate(5.0f);
}

void PhysicsComponent::rotate(float yaw, float pitch)
{
	mOrientation = mOrientation *
		Common::Quaternion::fromAxisAngle(Common::Vector3(0.0f, 1.0f, 0.0f), yaw);
	mPitch = mPitch *
		Common::Quaternion::fromAxisAngle(Common::Vector3(0.0f, 0.0f, 1.0f), pitch);
}

struct Ray {
	Common::Vector3 start;
	Common::Vector3 end;
};

class HittableComponent {
	public:
		HittableComponent() = default;
		HittableComponent(const PhysicsComponent* phys, float radius);
		bool hit(const Ray& ray) const;
		void die();
		bool hasDied() const;

	private:
		const PhysicsComponent* mPhys;
		float mRadius;
		bool mDied;
};

HittableComponent::HittableComponent(const PhysicsComponent* phys, float radius)
	: mPhys(phys),
	mRadius(radius),
	mDied(false)
{
}

bool HittableComponent::hit(const Ray& ray) const
{
	auto ret = Common::Math::segmentCircleIntersect(ray.start, ray.end, mPhys->getPosition(), mRadius);
	return ret;
}

void HittableComponent::die()
{
	mDied = true;
}

bool HittableComponent::hasDied() const
{
	return mDied;
}

class HitterComponent {
	public:
		HitterComponent() = default;
		HitterComponent(const PhysicsComponent* phys, unsigned int shooterID);
		void update(float dt);
		Ray getLastRay() const;
		float getSpeed() const;
		unsigned int getShooterID() const;

	private:
		const PhysicsComponent* mPhys;
		Common::Vector3 mPrevPos;
		Common::Vector3 mThisPos;
		unsigned int mShooterID;
};

HitterComponent::HitterComponent(const PhysicsComponent* phys, unsigned int shooterID)
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

float HitterComponent::getSpeed() const
{
	return mPhys->getVelocity().length();
}

unsigned int HitterComponent::getShooterID() const
{
	return mShooterID;
}

class BulletRenderer {
	public:
		BulletRenderer();
		BulletRenderer(Scene::Scene* scene, const HitterComponent* hit);
		void update(float dt);

	private:
		Scene::Scene* mScene;
		const HitterComponent* mHitter;
};

BulletRenderer::BulletRenderer()
{
}

BulletRenderer::BulletRenderer(Scene::Scene* scene, const HitterComponent* hit)
	: mScene(scene),
	mHitter(hit)
{
}

void BulletRenderer::update(float dt)
{
	auto r = mHitter->getLastRay();
	if(mHitter->getSpeed() > 10.0f) {
		mScene->addLine("Bullet line", r.start, r.end, Common::Color::Red);
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
		std::vector<PhysicsComponent> mPhysics;
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
	mPhysics.reserve(MAX_BULLETS);
	mHitters.reserve(MAX_BULLETS);
	mRenders.reserve(MAX_BULLETS);
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
	PhysicsComponent* ph = &mPhysics[mNumBullets];
	*ph = PhysicsComponent(mMap, 0.0f, 0.99f, 0.0f);
	ph->setPosition(pos + Common::Vector3(0.0f, 1.0f, 0.0f));
	ph->setOrientation(ori);
	ph->setVelocity(Common::Math::rotate3D(Scene::WorldForward, ori) * 100.0f);

	mHitters[mNumBullets] = HitterComponent(ph, shooterID);
	mRenders[mNumBullets] = BulletRenderer(mScene, &mHitters[mNumBullets]);

	mNumBullets++;
	weapon.shoot();
}

std::vector<unsigned int> Bullets::checkForHits(const std::vector<HittableComponent>& hittables)
{
	std::vector<unsigned int> ret;
	for(unsigned int j = 0; j < hittables.size(); j++) {
		for(unsigned int i = 0; i < mNumBullets; i++) {
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
		ShooterComponent(const PhysicsComponent* phys, Bullets* bullets, unsigned int shooterID);
		ShooterComponent() = default;
		void shoot();
		void update(float dt);

	private:
		const PhysicsComponent* mPhys;
		Bullets* mBullets;
		Weapon mWeapon;
		unsigned int mShooterID;
};

ShooterComponent::ShooterComponent(const PhysicsComponent* phys, Bullets* bullets, unsigned int shooterID)
	: mPhys(phys),
	mBullets(bullets),
	mShooterID(shooterID)
{
}

void ShooterComponent::shoot()
{
	if(mWeapon.canShoot()) {
		mBullets->shoot(mWeapon, mPhys->getPosition(), mPhys->getOrientation() * mPhys->getPitch(), mShooterID);
	}
}

void ShooterComponent::update(float dt)
{
	mWeapon.update(dt);
}

class InputComponent {
	public:
		InputComponent(PhysicsComponent* phys, ShooterComponent* shooter, HittableComponent* hit, bool player);
		InputComponent() = default;
		void update(float dt);
		bool handleKeyDown(float frameTime, SDLKey key);
		bool handleKeyUp(float frameTime, SDLKey key);
		bool handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev);
		bool handleMousePress(float frameTime, Uint8 button);

	private:
		PhysicsComponent* mPhys;
		ShooterComponent* mShooter;
		HittableComponent* mHittable;
		bool mPlayer;
		Common::Vector3 mInputAccel;
};

InputComponent::InputComponent(PhysicsComponent* phys, ShooterComponent* shooter, HittableComponent* hit, bool player)
	: mPhys(phys),
	mShooter(shooter),
	mHittable(hit),
	mPlayer(player)
{
}

void InputComponent::update(float dt)
{
	if(mHittable->hasDied())
		return;

	Common::Vector3 accel;
	if(!mPlayer) {
		accel = Common::Vector3(0.5f, 0.0f, 1.0f);
	} else {
		accel = Common::Math::rotate3D(mInputAccel, mPhys->getOrientation());
	}
	mPhys->addAcceleration(accel);
}

bool InputComponent::handleKeyDown(float frameTime, SDLKey key)
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

		default:
			break;
	}

	return false;
}

bool InputComponent::handleKeyUp(float frameTime, SDLKey key)
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

bool InputComponent::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	if(mHittable->hasDied())
		return false;

	mPhys->rotate(-ev.xrel * 0.02f, -ev.yrel * 0.02f);
	return false;
}

bool InputComponent::handleMousePress(float frameTime, Uint8 button)
{
	if(mHittable->hasDied())
		return false;

	if(button == SDL_BUTTON_LEFT) {
		mShooter->shoot();
	}
	return false;
}

class RenderComponent {
	public:
		RenderComponent(Scene::Scene& scene, const PhysicsComponent* phys, unsigned int num);
		RenderComponent() = default;
		void update(float dt);

	private:
		const PhysicsComponent* mPhys;
		Scene::MeshInstance* mMesh;
};

RenderComponent::RenderComponent(Scene::Scene& scene, const PhysicsComponent* phys, unsigned int num)
	: mPhys(phys)
{
	char name[256];
	snprintf(name, 255, "Soldier%d", num);
	mMesh = scene.addMeshInstance(name, "Cube", "Snow").get();
}

void RenderComponent::update(float dt)
{
	mMesh->setPosition(mPhys->getPosition());
}

class Soldiers {
	static const unsigned int MAX_SOLDIERS = 256;
	public:
		Soldiers(Scene::Scene& scene);
		void update(float dt);
		void addSoldiers(const WorldMap* wmap, Bullets* bullets);
		const Common::Vector3& getPlayerSoldierPosition() const;
		Common::Quaternion getPlayerSoldierOrientation() const;
		InputComponent& getPlayerInputComponent();
		std::vector<HittableComponent> getHittables() const;
		void processHits(const std::vector<unsigned int>& hits);

	private:
		unsigned int mNumSoldiers;
		std::vector<InputComponent> mInputs;
		std::vector<PhysicsComponent> mPhysics;
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
	mInputs.reserve(MAX_SOLDIERS);
	mPhysics.reserve(MAX_SOLDIERS);
	mRenders.reserve(MAX_SOLDIERS);
	mShooters.reserve(MAX_SOLDIERS);
	mHittables.reserve(MAX_SOLDIERS);
}

void Soldiers::update(float dt)
{
	for(unsigned int i = 0; i < mNumSoldiers; i++) {
		mInputs[i].update(dt);
	}
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
	const unsigned int numSoldiers = 10;

	mPlayerSoldierIndex = 0;
	for(int i = 0; i < numSoldiers; i++) {
		mPhysics[i] = PhysicsComponent(wmap, 5.0f, 0.95f, 1.0f);
		mShooters[i] = ShooterComponent(&mPhysics[i], bullets, i);
		mHittables[i] = HittableComponent(&mPhysics[i], 1.5f);
		mInputs[i] = InputComponent(&mPhysics[i], &mShooters[i], &mHittables[i], i == mPlayerSoldierIndex);
		mRenders[i] = RenderComponent(mScene, &mPhysics[i], i);
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

	return mPhysics[mPlayerSoldierIndex].getOrientation() * mPhysics[mPlayerSoldierIndex].getPitch();
}

InputComponent& Soldiers::getPlayerInputComponent()
{
	assert(mPlayerSoldierIndex < mNumSoldiers);

	return mInputs[mPlayerSoldierIndex];
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

class World {
	public:
		World(Scene::Scene& scene);
		void addSoldiers();
		void createMap();
		void update(float dt);
		bool handleKeyDown(float frameTime, SDLKey key);
		bool handleKeyUp(float frameTime, SDLKey key);
		bool handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev);
		bool handleMousePress(float frameTime, Uint8 button);

	private:
		WorldMap mMap;
		Soldiers mSoldiers;
		Bullets mBullets;
		Scene::Scene& mScene;
		bool mObserverMode;
		Scene::Camera& mCamera;
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

void World::addSoldiers()
{
	mSoldiers.addSoldiers(&mMap, &mBullets);
}

void World::createMap()
{
	mMap.create();
}

void World::update(float dt)
{
	if(mObserverMode)
		mCamera.applyMovementKeys(dt);

	mSoldiers.update(dt);
	mBullets.update(dt);

	auto hits = mBullets.checkForHits(mSoldiers.getHittables());
	mSoldiers.processHits(hits);

	if(!mObserverMode) {
		mCamera.setPosition(mSoldiers.getPlayerSoldierPosition() + Common::Vector3(0.0f, 1.0f, 0.0f));

		auto ori = mSoldiers.getPlayerSoldierOrientation();
		auto tgt = Common::Math::rotate3D(Scene::WorldForward, ori);
		auto up = Common::Math::rotate3D(Scene::WorldUp, ori);
		mCamera.lookAt(tgt, up);
	}
}

bool World::handleKeyDown(float frameTime, SDLKey key)
{
	if(key == SDLK_F1) {
		mObserverMode = !mObserverMode;
		return false;
	}

	if(mObserverMode) {
		auto it = mControls.find(key);
		if(it != mControls.end()) {
			it->second(0.1f);
			return false;
		} else {
			if(key == SDLK_ESCAPE) {
				return true;
			}
		}
		return false;
	} else {
		return mSoldiers.getPlayerInputComponent().handleKeyDown(frameTime, key);
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
		return mSoldiers.getPlayerInputComponent().handleKeyUp(frameTime, key);
	}
}

bool World::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	if(mObserverMode) {
		if(SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(1)) {
			mCamera.rotate(-ev.xrel * 0.02f, -ev.yrel * 0.02f);
		}
		return false;
	} else {
		return mSoldiers.getPlayerInputComponent().handleMouseMotion(frameTime, ev);
	}
}

bool World::handleMousePress(float frameTime, Uint8 button)
{
	if(mObserverMode) {
		return false;
	} else {
		return mSoldiers.getPlayerInputComponent().handleMousePress(frameTime, button);
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
	mScene.addModel("Cube", "share/textured-cube.obj");
	mScene.addModel("Tree", "share/tree.obj");
	mScene.addTexture("Snow", "share/snow.jpg");

	mScene.getAmbientLight().setState(true);
	mScene.getAmbientLight().setColor(Common::Color(127, 127, 127));

	mScene.getDirectionalLight().setState(true);
	mScene.getDirectionalLight().setDirection(Common::Vector3(0.5f, -1.0f, 0.5f));
	mScene.getDirectionalLight().setColor(Common::Vector3(0.9f, 0.9f, 0.9f));

	mWorld.createMap();
	mWorld.addSoldiers();
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
