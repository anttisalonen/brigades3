#include <iostream>
#include <sstream>
#include <memory>
#include <vector>
#include <map>
#include <random>

#include <string.h>
#include <assert.h>

#include <sscene/Scene.h>

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

class PhysicsComponent {
	public:
		PhysicsComponent(WorldMap& wmap);
		void update(float dt);
		void addAcceleration(const Common::Vector3& vec);
		const Common::Vector3& getPosition() const { return mPosition; }
		const Common::Quaternion& getOrientation() const { return mOrientation; }
		void rotate(float yaw, float pitch);

	private:
		Common::Vector3 mPosition;
		Common::Vector3 mVelocity;
		Common::Vector3 mAcceleration;
		Common::Quaternion mOrientation;
		WorldMap& mMap;
};

PhysicsComponent::PhysicsComponent(WorldMap& wmap)
	: mMap(wmap)
{
}

void PhysicsComponent::update(float dt)
{
	mVelocity += mAcceleration * dt;
	mVelocity.truncate(5.0f);

	if(dt) {
		mVelocity = mVelocity * 0.95f;
	}

	mPosition += mVelocity * dt;
	mPosition.y = mMap.getHeightAt(mPosition.x, mPosition.z);

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
}

class InputComponent {
	public:
		InputComponent(PhysicsComponent& phys, bool player);
		void update(float dt);
		bool handleKeyDown(float frameTime, SDLKey key);
		bool handleKeyUp(float frameTime, SDLKey key);
		bool handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev);

	private:
		PhysicsComponent& mPhys;
		bool mPlayer;
		Common::Vector3 mInputAccel;
};

InputComponent::InputComponent(PhysicsComponent& phys, bool player)
	: mPhys(phys),
	mPlayer(player)
{
}

void InputComponent::update(float dt)
{
	Common::Vector3 accel;
	if(!mPlayer) {
		accel = Common::Vector3(0.5f, 0.0f, 1.0f);
	} else {
		accel = Common::Math::rotate3D(mInputAccel, mPhys.getOrientation());
	}
	mPhys.addAcceleration(accel);
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
	mPhys.rotate(-ev.xrel * 0.02f, -ev.yrel * 0.02f);
	return false;
}

class RenderComponent {
	public:
		RenderComponent(Scene::Scene& scene, const PhysicsComponent& phys, unsigned int num);
		void update(float dt);

	private:
		const PhysicsComponent& mPhys;
		Scene::MeshInstance* mMesh;
};

RenderComponent::RenderComponent(Scene::Scene& scene, const PhysicsComponent& phys, unsigned int num)
	: mPhys(phys)
{
	char name[256];
	snprintf(name, 255, "Soldier%d", num);
	mMesh = scene.addMeshInstance(name, "Cube", "Snow").get();
}

void RenderComponent::update(float dt)
{
	mMesh->setPosition(mPhys.getPosition());
}

class Soldiers {
	public:
		Soldiers(Scene::Scene& scene);
		void update(float dt);
		void addSoldiers(WorldMap& wmap);
		const Common::Vector3& getPlayerSoldierPosition() const;
		const Common::Quaternion& getPlayerSoldierOrientation() const;
		InputComponent& getPlayerInputComponent();

	private:
		unsigned int mNumSoldiers;
		std::vector<InputComponent> mInputs;
		std::vector<PhysicsComponent> mPhysics;
		std::vector<RenderComponent> mRenders;

		Scene::Scene& mScene;
		unsigned int mPlayerSoldierIndex;
};

Soldiers::Soldiers(Scene::Scene& scene)
	: mNumSoldiers(0),
	mScene(scene),
	mPlayerSoldierIndex(0)
{
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
		mRenders[i].update(dt);
	}
}

void Soldiers::addSoldiers(WorldMap& wmap)
{
	for(int i = 0; i < 10; i++) {
		mPhysics.emplace_back(wmap);
	}

	for(int i = 0; i < 10; i++) {
		mInputs.emplace_back(mPhysics[i], i == 0);
	}

	for(int i = 0; i < 10; i++) {
		mRenders.emplace_back(mScene, mPhysics[i], i);
	}

	mPlayerSoldierIndex = 0;

	mNumSoldiers = 10;
}

const Common::Vector3& Soldiers::getPlayerSoldierPosition() const
{
	assert(mPlayerSoldierIndex < mNumSoldiers);

	return mPhysics[mPlayerSoldierIndex].getPosition();
}

const Common::Quaternion& Soldiers::getPlayerSoldierOrientation() const
{
	assert(mPlayerSoldierIndex < mNumSoldiers);

	return mPhysics[mPlayerSoldierIndex].getOrientation();
}

InputComponent& Soldiers::getPlayerInputComponent()
{
	assert(mPlayerSoldierIndex < mNumSoldiers);

	return mInputs[mPlayerSoldierIndex];
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

	private:
		WorldMap mMap;
		Soldiers mSoldiers;
		Scene::Scene& mScene;
};

World::World(Scene::Scene& scene)
	: mMap(scene),
	mSoldiers(scene),
	mScene(scene)
{
}

void World::addSoldiers()
{
	mSoldiers.addSoldiers(mMap);
}

void World::createMap()
{
	mMap.create();
}

void World::update(float dt)
{
	mSoldiers.update(dt);

	auto& cam = mScene.getDefaultCamera();
	cam.setPosition(mSoldiers.getPlayerSoldierPosition() + Common::Vector3(0.0f, 1.0f, 0.0f));

	auto ori = mSoldiers.getPlayerSoldierOrientation();
	auto tgt = Common::Math::rotate3D(Scene::WorldForward, ori);
	auto up = Common::Math::rotate3D(Scene::WorldUp, ori);
	cam.lookAt(tgt, up);
}

bool World::handleKeyDown(float frameTime, SDLKey key)
{
	return mSoldiers.getPlayerInputComponent().handleKeyDown(frameTime, key);
}

bool World::handleKeyUp(float frameTime, SDLKey key)
{
	return mSoldiers.getPlayerInputComponent().handleKeyUp(frameTime, key);
}

bool World::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	return mSoldiers.getPlayerInputComponent().handleMouseMotion(frameTime, ev);
}

class AppDriver : public Common::Driver {
	public:
		AppDriver();
		virtual void drawFrame() override;
		virtual bool handleKeyDown(float frameTime, SDLKey key) override;
		virtual bool handleKeyUp(float frameTime, SDLKey key) override;
		virtual bool handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev) override;
		virtual bool prerenderUpdate(float frameTime) override;

	private:
		void handleMouseMove(float dx, float dy);

		Scene::Scene mScene;
		Scene::Camera& mCamera;
		World mWorld;

		std::map<SDLKey, std::function<void (float)>> mControls;
		bool mObserverMode;
};

AppDriver::AppDriver()
	: Common::Driver(800, 600, "Army"),
	mScene(800, 600),
	mCamera(mScene.getDefaultCamera()),
	mWorld(mScene),
	mObserverMode(false)
{
	mScene.addModel("Cube", "share/textured-cube.obj");
	mScene.addModel("Tree", "share/tree.obj");
	mScene.addTexture("Snow", "share/snow.jpg");

	mScene.getAmbientLight().setState(true);
	mScene.getAmbientLight().setColor(Common::Color(127, 127, 127));

	mScene.getDirectionalLight().setState(true);
	mScene.getDirectionalLight().setDirection(Common::Vector3(0.5f, -1.0f, 0.5f));
	mScene.getDirectionalLight().setColor(Common::Vector3(0.9f, 0.9f, 0.9f));

	mControls[SDLK_w] = [&] (float p) { mCamera.setForwardMovement(p); };
	mControls[SDLK_q] = [&] (float p) { mCamera.setUpwardsMovement(p); };
	mControls[SDLK_d] = [&] (float p) { mCamera.setSidewaysMovement(p); };
	mControls[SDLK_s] = [&] (float p) { mCamera.setForwardMovement(-p); };
	mControls[SDLK_e] = [&] (float p) { mCamera.setUpwardsMovement(-p); };
	mControls[SDLK_a] = [&] (float p) { mCamera.setSidewaysMovement(-p); };

	mWorld.createMap();
	mWorld.addSoldiers();

	mObserverMode = false;
}

bool AppDriver::handleKeyDown(float frameTime, SDLKey key)
{
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
	} else {
		return mWorld.handleKeyDown(frameTime, key);
	}

	return false;
}

bool AppDriver::handleKeyUp(float frameTime, SDLKey key)
{
	if(mObserverMode) {
		auto it = mControls.find(key);
		if(it != mControls.end()) {
			it->second(0.0f);
		}
	} else {
		return mWorld.handleKeyUp(frameTime, key);
	}

	return false;
}

bool AppDriver::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	if(mObserverMode) {
		if(SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(1)) {
			handleMouseMove(ev.xrel, ev.yrel);
		}
	} else {
		return mWorld.handleMouseMotion(frameTime, ev);
	}

	return false;
}

void AppDriver::handleMouseMove(float dx, float dy)
{
	mCamera.rotate(-dx * 0.02f, -dy * 0.02f);
}

bool AppDriver::prerenderUpdate(float frameTime)
{
	if(mObserverMode)
		mCamera.applyMovementKeys(frameTime);

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
