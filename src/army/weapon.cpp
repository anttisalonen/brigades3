#include <assert.h>

#include "weapon.hpp"

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


