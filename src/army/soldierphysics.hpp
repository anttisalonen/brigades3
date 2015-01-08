#ifndef BRIGADES_SOLDIERPHYSICS_HPP
#define BRIGADES_SOLDIERPHYSICS_HPP

#include <common/Vector3.h>
#include <common/Quaternion.h>

#include "worldmap.hpp"
#include "physicscommon.hpp"

class SoldierPhysics {
	public:
		SoldierPhysics() = default;
		SoldierPhysics(const WorldMap* wmap, float maxvel, float damping, float friction);
		void update(float dt);
		void addAcceleration(const Common::Vector3& vec);
		const Common::Vector3& getPosition() const { return mPosition; }
		const Common::Vector3& getVelocity() const { return mVelocity; }
		const Common::Quaternion& getOrientation() const { return mOrientation; }
		Common::Quaternion getAimPitch() const;
		void rotate(float yaw, float pitch);

		void setPosition(const Common::Vector3& pos) { mPosition = pos; }
		void setOrientation(const Common::Quaternion& ori) { mOrientation = ori; }
		void setVelocity(const Common::Vector3& v) { mVelocity = v; }

		void setAiming(bool a);
		bool isAiming() const;

		static constexpr float RunAcceleration = 10.0f;

	private:
		void checkTreeCollision(const Common::Vector3& oldpos);
		void checkLandCollision(const Common::Vector3& oldpos);
		void checkWallCollision(const Common::Vector3& oldpos);

		Common::Vector3 mPosition;
		Common::Vector3 mVelocity;
		Common::Vector3 mAcceleration;
		Common::Quaternion mOrientation;
		float mAimPitch;
		const WorldMap* mMap;
		float mMaxVel;
		float mDamping;
		float mFriction;
		bool mAiming;

		friend class PhysicsCommon<SoldierPhysics>;
};

#endif

