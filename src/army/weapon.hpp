#ifndef BRIGADES_WEAPON_HPP
#define BRIGADES_WEAPON_HPP

#include <common/Clock.h>

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

#endif
