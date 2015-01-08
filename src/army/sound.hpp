#ifndef BRIGADES_SOUND_HPP
#define BRIGADES_SOUND_HPP

#include <common/Vector3.h>

#include <SDL/SDL_mixer.h>

class Sound {
	public:
		static void init();
		static void play(const Common::Vector3& pos);
		static void setCamera(const Common::Vector3& pos, const Common::Vector3& lookdir);

	private:
		static Mix_Chunk* mRifle;
		static unsigned int mNextChannel;
		static Common::Vector3 mCamPos;
		static Common::Vector3 mCamLook;
};

#endif

