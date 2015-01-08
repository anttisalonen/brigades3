#ifndef BRIGADES_BULLETPHYSICS_HPP
#define BRIGADES_BULLETPHYSICS_HPP

#include <common/Vector3.h>

#include "worldmap.hpp"
#include "physicscommon.hpp"

class BulletPhysics {
	public:
		BulletPhysics() = default;
		BulletPhysics(const WorldMap* wmap, float maxvel, float damping, float friction);
		void update(float dt);
		const Common::Vector3& getPosition() const { return mPosition; }
		const Common::Vector3& getVelocity() const { return mVelocity; }

		void setPosition(const Common::Vector3& pos) { mPosition = pos; }
		void setVelocity(const Common::Vector3& v) { mVelocity = v; }
		bool isActive() const { return mActive; }

	private:
		void checkTreeCollision(const Common::Vector3& oldpos);
		void checkLandCollision(const Common::Vector3& oldpos);
		void checkWallCollision(const Common::Vector3& oldpos);

		Common::Vector3 mPosition;
		Common::Vector3 mVelocity;
		Common::Vector3 mAcceleration;
		const WorldMap* mMap;
		float mMaxVel;
		float mDamping;
		bool mActive;

		friend class PhysicsCommon<BulletPhysics>;
};

#endif

