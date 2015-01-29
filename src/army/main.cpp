#include <iostream>
#include <memory>
#include <map>
#include <sstream>

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
	AIConstants AIConstants = { 0.5f, 0.5f, false };
	unsigned int WorldRandomSeed = 0;
	unsigned int GameRandomSeed = 0;
	unsigned int AIRandomSeed = 0;
	float DeterministicStep = 0;
	bool Observer = false;
	bool NoGUI = false;
};

class World {
	public:
		World(Scene::Scene& scene, const Constants& constants);
		void init();
		bool update(float dt);
		bool handleKeyDown(float frameTime, SDLKey key);
		bool handleKeyUp(float frameTime, SDLKey key);
		bool handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev);
		bool handleMousePress(float frameTime, Uint8 button);

	private:
		void setCameraForSoldier(unsigned int soldierIndex);
		void updateUIMessages(float dt);
		bool checkForWin(float dt);
		void showMessage(const std::string& msg);

		WorldMap mMap;
		Soldiers mSoldiers;
		Bullets mBullets;
		AI mAI;
		Scene::Scene& mScene;
		const bool mObserverMode;
		unsigned int mObservedSoldier = UINT_MAX;
		Scene::Camera& mCamera;
		PlayerInput mPlayerInput;
		std::map<SDLKey, std::function<void (float)>> mControls;
		Constants mConstants;
		unsigned int mCurrentDied = 0;
		Common::Countdown mDiedMessageTimer;
		Common::Countdown mEndRoundTimer;
		bool mWinnerFound = false;
};

World::World(Scene::Scene& scene, const Constants& constants)
	: mMap(constants.NoGUI ? nullptr : &scene),
	mSoldiers(constants.NoGUI ? nullptr : &scene, constants.Observer ? UINT_MAX : 0),
	mBullets(&mMap, constants.NoGUI ? nullptr : &scene),
	mScene(scene),
	mObserverMode(constants.Observer),
	mCamera(mScene.getDefaultCamera()),
	mConstants(constants),
	mDiedMessageTimer(10.0f),
	mEndRoundTimer(10.0f)
{
	mDiedMessageTimer.clear();
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
	if(!mObserverMode) {
		mPlayerInput = PlayerInput(&mSoldiers);
	}
	mAI = AI(&mMap, &mSoldiers, mConstants.AIConstants, mConstants.NoGUI ? nullptr : &mScene);
	mAI.init();

	if(mObserverMode) {
		auto w = mMap.getWidth() * 0.5f;
		mCamera.setPosition(mMap.pointToVec(w, w) + Common::Vector3(0.0f, 20.0f, 0.0f));
	}
}

bool World::update(float dt)
{
	mSoldiers.update(dt);
	if(!mObserverMode) {
		mPlayerInput.update(dt);
	}
	mAI.update(dt);
	mBullets.update(dt);

	auto hits = mBullets.checkForHits(mSoldiers.getHittables());
	mSoldiers.processHits(hits);

	updateUIMessages(dt);

	if(!mConstants.NoGUI) {
		if(mObserverMode) {
			mScene.setOverlayEnabled("Sight", false);
			if(mObservedSoldier == UINT_MAX) {
				mCamera.applyMovementKeys(dt);
			} else {
				setCameraForSoldier(mObservedSoldier);
				mScene.setFOV(90.0f);
			}
		} else {
			setCameraForSoldier(mSoldiers.getPlayerSoldierIndex());
			mScene.setFOV(mPlayerInput.getFOV());
		}
		Sound::setCamera(mCamera.getPosition(), mCamera.getTargetVector());
	}

	if(checkForWin(dt)) {
		return true;
	}
	return false;
}

void World::updateUIMessages(float dt)
{
	std::stringstream msg;
	unsigned int currentDied = 0;
	for(const auto& h : mSoldiers.getHittables()) {
		if(h.hasDied())
			currentDied++;
	}

	mDiedMessageTimer.doCountdown(dt);
	if(currentDied != mCurrentDied) {
		mCurrentDied = currentDied;
		msg << currentDied << " down, " << (mSoldiers.getNumSoldiers() - currentDied) << " to go!";
		showMessage(msg.str());
		mDiedMessageTimer.rewind();
	} else if(mDiedMessageTimer.running() && mDiedMessageTimer.check()) {
		if(!mConstants.NoGUI) {
			mScene.setOverlayEnabled("DiedMessage", false);
		}
	}
}

void World::showMessage(const std::string& msg)
{
	if(!mConstants.NoGUI) {
		mScene.addOverlayText("DiedMessage", msg, Common::Color::Red,
				1.0f, 0.5f, 0.08f, true);
		mScene.setOverlayDepth("DiedMessage", 1.0f);
	} else {
		std::cout << msg << "\n";
	}
}

bool World::checkForWin(float dt)
{
	if(mCurrentDied >= mSoldiers.getNumSoldiers() - 1) {
		std::stringstream msg;

		if(!mWinnerFound && mCurrentDied == mSoldiers.getNumSoldiers() - 1) {
			for(unsigned int i = 0; i < mSoldiers.getNumSoldiers(); i++) {
				if(!mSoldiers.getHittable(i)->hasDied()) {
					msg << mSoldiers.getName(i) << " has won!";
					showMessage(msg.str());
					mWinnerFound = true;
					break;
				}
			}
		}
		mEndRoundTimer.doCountdown(dt);

		if(mEndRoundTimer.check()) {
			return true;
		}
	}
	return false;
}

void World::setCameraForSoldier(unsigned int soldierIndex)
{
	auto ori = mSoldiers.getSoldierOrientation(soldierIndex);
	auto tgt = Common::Math::rotate3D(Scene::WorldForward, ori);
	auto up = Common::Math::rotate3D(Scene::WorldUp, ori);

	if(!mSoldiers.soldierIsAlive(soldierIndex)) {
		up = Common::Math::rotate3D(up, HALF_PI, Common::Vector3(1.0f, 0.0f, 0.0f));
	}

	mCamera.setPosition(mSoldiers.getSoldierPosition(soldierIndex) +
			Common::Vector3(0.0f, 1.7f, 0.0f) +
			tgt.normalized() * 0.5f);
	mCamera.lookAt(tgt, up);
	mScene.setOverlayEnabled("Sight", mSoldiers.getSoldierAiming(soldierIndex));
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

	if(key == SDLK_F2 && mObserverMode) {
		if(mObservedSoldier == UINT_MAX) {
			mObservedSoldier = 0;
		} else {
			mObservedSoldier++;
		}
		if(mObservedSoldier >= mSoldiers.getNumSoldiers()) {
			mObservedSoldier = UINT_MAX;
		}
		std::cout << "Observed soldier: " << mObservedSoldier << "\n";
		return false;
	}
	if(key == SDLK_F3 && mObserverMode) {
		if(mObservedSoldier == UINT_MAX) {
			if(mSoldiers.getNumSoldiers() > 0) {
				mObservedSoldier = mSoldiers.getNumSoldiers() - 1;
			}
		} else if(mObservedSoldier == 0) {
			mObservedSoldier = UINT_MAX;
		} else {
			mObservedSoldier--;
		}
		std::cout << "Observed soldier: " << mObservedSoldier << "\n";
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
		if(!mObserverMode) {
			return mPlayerInput.handleKeyDown(frameTime, key);
		} else {
			return false;
		}
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
		if(!mObserverMode) {
			return mPlayerInput.handleKeyUp(frameTime, key);
		} else {
			return false;
		}
	}
}

bool World::handleMouseMotion(float frameTime, const SDL_MouseMotionEvent& ev)
{
	if(mObserverMode) {
		mCamera.rotate(-ev.xrel * 0.02f, -ev.yrel * 0.02f);
		return false;
	} else {
		if(!mObserverMode) {
			return mPlayerInput.handleMouseMotion(frameTime, ev);
		} else {
			return false;
		}
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
		if(!mObserverMode) {
			return mPlayerInput.handleMousePress(frameTime, button);
		} else {
			return false;
		}
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
		const Constants mConstants;
};

AppDriver::AppDriver(const Constants& constants)
	: Common::Driver(constants.NoGUI ? 0 : 800, constants.NoGUI ? 0 : 600, "Army"),
	mScene(800, 600),
	mWorld(mScene, constants),
	mConstants(constants)
{
	if(!mConstants.NoGUI) {
		mScene.init();
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

		mScene.enableText("share/DejaVuSans.ttf");
	}

	mWorld.init();
	if(!mConstants.NoGUI) {
		Sound::init();
	}
	WindowFocus::setWindowFocus(true);
}

bool AppDriver::handleKeyDown(float frameTime, SDLKey key)
{
	if(key == SDLK_MINUS || key == SDLK_KP_MINUS) {
		auto ta = getTimeAcceleration();
		if(ta > 1)
			ta /= 2;
		setTimeAcceleration(ta);
		std::cout << "Time acceleration: " << ta << "\n";
	}
	if(key == SDLK_PLUS || key == SDLK_KP_PLUS) {
		auto ta = getTimeAcceleration();
		if(ta < 256)
			ta *= 2;
		setTimeAcceleration(ta);
		std::cout << "Time acceleration: " << ta << "\n";
	}

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
	return mWorld.update(frameTime);
}

void AppDriver::drawFrame()
{
	if(!mConstants.NoGUI) {
		mScene.render();
	}
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
		} else if(!strcmp(argv[i], "--aidebug")) {
			c.AIConstants.AIDebug = true;
		} else if(!strcmp(argv[i], "--nogui")) {
			c.NoGUI = true;
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
