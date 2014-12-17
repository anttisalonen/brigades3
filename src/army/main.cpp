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

	private:
		Common::Vector3 mPosition;
		Common::Vector3 mVelocity;
		Common::Vector3 mAcceleration;
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

	mPosition += mVelocity * dt;
	mPosition.y = mMap.getHeightAt(mPosition.x, mPosition.z);

	mAcceleration = Common::Vector3();
}

void PhysicsComponent::addAcceleration(const Common::Vector3& vec)
{
	mAcceleration += vec;
	mAcceleration.truncate(5.0f);
}

class InputComponent {
	public:
		InputComponent(PhysicsComponent& phys);
		void update(float dt);

	private:
		PhysicsComponent& mPhys;
};

InputComponent::InputComponent(PhysicsComponent& phys)
	: mPhys(phys)
{
}

void InputComponent::update(float dt)
{
	mPhys.addAcceleration(Common::Vector3(1.0f, 0.0f, 1.0f));
}

class RenderComponent {
	public:
		RenderComponent(Scene::Scene& scene, const PhysicsComponent& phys);
		void update(float dt);

	private:
		const PhysicsComponent& mPhys;
		Scene::MeshInstance* mMesh;
};

RenderComponent::RenderComponent(Scene::Scene& scene, const PhysicsComponent& phys)
	: mPhys(phys)
{
	mMesh = scene.addMeshInstance("Cube1", "Cube", "Snow").get();
}

void RenderComponent::update(float dt)
{
	mMesh->setPosition(mPhys.getPosition());
}

class Soldiers {
	public:
		Soldiers(Scene::Scene& scene);
		void update(float dt);
		void addSoldier(WorldMap& wmap);

	private:
		unsigned int mNumSoldiers;
		std::vector<InputComponent> mInputs;
		std::vector<PhysicsComponent> mPhysics;
		std::vector<RenderComponent> mRenders;

		Scene::Scene& mScene;
};

Soldiers::Soldiers(Scene::Scene& scene)
	: mNumSoldiers(0),
	mScene(scene)
{
}

void Soldiers::update(float dt)
{
	for(int i = 0; i < mNumSoldiers; i++) {
		mInputs[i].update(dt);
	}
	for(int i = 0; i < mNumSoldiers; i++) {
		mPhysics[i].update(dt);
	}
	for(int i = 0; i < mNumSoldiers; i++) {
		mRenders[i].update(dt);
	}
}

void Soldiers::addSoldier(WorldMap& wmap)
{
	mPhysics.push_back(PhysicsComponent(wmap));
	mInputs.push_back(InputComponent(mPhysics[mNumSoldiers]));
	mRenders.push_back(RenderComponent(mScene, mPhysics[mNumSoldiers]));
	mNumSoldiers++;
}

class World {
	public:
		World(Scene::Scene& scene);
		void addSoldier();
		void createMap();
		void update(float dt);

	private:
		WorldMap mMap;
		Soldiers mSoldiers;
};

World::World(Scene::Scene& scene)
	: mMap(scene),
	mSoldiers(scene)
{
}

void World::addSoldier()
{
	mSoldiers.addSoldier(mMap);
}

void World::createMap()
{
	mMap.create();
}

void World::update(float dt)
{
	mSoldiers.update(dt);
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
};

AppDriver::AppDriver()
	: Common::Driver(800, 600, "Army"),
	mScene(800, 600),
	mCamera(mScene.getDefaultCamera()),
	mWorld(mScene)
{
	mScene.addModel("Cube", "share/textured-cube.obj");
	mScene.addModel("Tree", "share/tree.obj");
	mScene.addTexture("Snow", "share/snow.jpg");

	mCamera.setPosition(Common::Vector3(1.9f, 1.9f, 4.2f));
	mCamera.rotate(Common::Math::degreesToRadians(90), 0);

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
	mWorld.addSoldier();
}

bool AppDriver::handleKeyDown(float frameTime, SDLKey key)
{
	auto it = mControls.find(key);
	if(it != mControls.end()) {
		it->second(0.1f);
	} else {
		if(key == SDLK_ESCAPE) {
			return true;
		}
		else if(key == SDLK_p) {
			std::cout << "Up: " << mCamera.getUpVector() << "\n";
			std::cout << "Target: " << mCamera.getTargetVector() << "\n";
			std::cout << "Position: " << mCamera.getPosition() << "\n";
		}
	}

	return false;
}

bool AppDriver::handleKeyUp(float frameTime, SDLKey key)
{
	auto it = mControls.find(key);
	if(it != mControls.end()) {
		it->second(0.0f);
	}
	return false;
}

bool AppDriver::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	if(SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(1)) {
		handleMouseMove(ev.xrel, ev.yrel);
	}
	return false;
}

void AppDriver::handleMouseMove(float dx, float dy)
{
	mCamera.rotate(-dx * 0.02f, -dy * 0.02f);
}

bool AppDriver::prerenderUpdate(float frameTime)
{
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
