#include <iostream>
#include <memory>
#include <map>

#include <string.h>
#include <stdlib.h>

#include <sscene/Scene.h>

#include <common/Math.h>
#include <common/DriverFramework.h>
#include <common/Random.h>

#include "sound.hpp"
#include "worldmap.hpp"
#include "bullets.hpp"
#include "soldiers.hpp"
#include "playerinput.hpp"
#include "ai.hpp"

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


struct Constants {
	unsigned int NumSoldiers = 4;
	AIConstants AIConstants = { 0.5f, 0.5f };
	unsigned int WorldRandomSeed = 0;
	unsigned int GameRandomSeed = 0;
	unsigned int AIRandomSeed = 0;
	float DeterministicStep = 0;
	bool Observer = false;
};

class World {
	public:
		World(Scene::Scene& scene, const Constants& constants);
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
		Constants mConstants;
};

World::World(Scene::Scene& scene, const Constants& constants)
	: mMap(scene),
	mSoldiers(scene),
	mBullets(&mMap, &scene),
	mScene(scene),
	mObserverMode(constants.Observer),
	mCamera(mScene.getDefaultCamera()),
	mConstants(constants)
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
	mSoldiers.addSoldiers(&mMap, &mBullets, mConstants.NumSoldiers);
	mPlayerInput = PlayerInput(&mSoldiers);
	mAI = AI(&mMap, &mSoldiers, mConstants.AIConstants);
	mAI.init();

	if(mObserverMode) {
		auto w = mMap.getWidth() * 0.5f;
		mCamera.setPosition(mMap.pointToVec(w, w) + Common::Vector3(0.0f, 20.0f, 0.0f));
	}
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
		AppDriver(const Constants& constants);
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

AppDriver::AppDriver(const Constants& constants)
	: Common::Driver(800, 600, "Army"),
	mScene(800, 600),
	mWorld(mScene, constants)
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
		App(const Constants& c);
		void go();

	private:
		AppDriver mDriver;
};

App::App(const Constants& c)
	: mDriver(c)
{
	if(c.DeterministicStep) {
		mDriver.setFixedTime(c.DeterministicStep, false);
	}
}

void App::go()
{
	mDriver.run();
}

int main(int argc, char** argv)
{
	Constants c;
	for(int i = 1; i < argc; i++) {
		if(!strcmp(argv[i], "--soldiers")) {
			c.NumSoldiers = atoi(argv[++i]);
		} else if(!strcmp(argv[i], "--shotskill")) {
			c.AIConstants.AvgShootingSkill = atof(argv[++i]);
			c.AIConstants.MaxShootingSkillVariation = 0.0f;
		} else if(!strcmp(argv[i], "--worldseed")) {
			c.WorldRandomSeed = atoi(argv[++i]);
		} else if(!strcmp(argv[i], "--gameseed")) {
			c.GameRandomSeed = atoi(argv[++i]);
		} else if(!strcmp(argv[i], "--aiseed")) {
			c.AIRandomSeed = atoi(argv[++i]);
		} else if(!strcmp(argv[i], "--deterministic")) {
			c.DeterministicStep = atoi(argv[++i]);
		} else if(!strcmp(argv[i], "--observer")) {
			c.Observer = true;
		} else {
			fprintf(stderr, "Unknown option \"%s\"\n", argv[i]);
			return 1;
		}
	}

	int timeseed = time(NULL);
	std::cout << "Using time seed " << timeseed << "\n";
	if(!c.WorldRandomSeed) {
		c.WorldRandomSeed = timeseed;
	}
	if(!c.GameRandomSeed) {
		c.GameRandomSeed = timeseed;
	}
	if(!c.AIRandomSeed) {
		c.AIRandomSeed = timeseed;
	}

	Random::seed(Random::SourceWorld, c.WorldRandomSeed);
	Random::seed(Random::SourceGame, c.GameRandomSeed);
	Random::seed(Random::SourceAI, c.AIRandomSeed);

	App a(c);
	a.go();
	return 0;
}
