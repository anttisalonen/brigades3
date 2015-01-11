#include <common/Math.h>

#include "playerinput.hpp"

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
	mPhys->addMovementAcceleration(accel * SoldierPhysics::RunAcceleration);
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

	mPhys->addRotation(-ev.xrel * coeff, -ev.yrel * coeff);
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


