#ifndef BRIGADES_PLAYERINPUT_HPP
#define BRIGADES_PLAYERINPUT_HPP

#include <SDL.h>

#include <common/Vector3.h>

#include "soldierphysics.hpp"
#include "hittable.hpp"
#include "soldiers.hpp"

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

#endif

