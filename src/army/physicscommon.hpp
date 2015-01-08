#ifndef BRIGADES_PHYSICSCOMMON_HPP
#define BRIGADES_PHYSICSCOMMON_HPP

template <typename Physics>
class PhysicsCommon {
	public:
		static void update(float dt, Physics& p);
};

template <typename Physics>
void PhysicsCommon<Physics>::update(float dt, Physics& p)
{
	// add forces
	p.mAcceleration.y -= 10.0f;

	// update position
	auto oldpos = p.mPosition;
	p.mPosition += p.mVelocity * dt;

	// update velocity
	p.mVelocity += p.mAcceleration * dt;
	if(p.mMaxVel) {
		p.mVelocity.truncate(p.mMaxVel);
	}
	if(dt && p.mDamping) {
		p.mVelocity = p.mVelocity * pow(p.mDamping, dt);
	}

	p.mAcceleration = Common::Vector3();

	p.checkTreeCollision(oldpos);
	p.checkLandCollision(oldpos);
	p.checkWallCollision(oldpos);
}

#endif

