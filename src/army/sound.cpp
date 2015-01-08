#include <common/Vector2.h>
#include <common/Math.h>

#include "sound.hpp"

Mix_Chunk* Sound::mRifle = nullptr;
unsigned int Sound::mNextChannel = 0;
Common::Vector3 Sound::mCamPos;
Common::Vector3 Sound::mCamLook;

void Sound::init()
{
	int flags=MIX_INIT_MP3;
	int initted=Mix_Init(flags);
	if((initted & flags) != flags) {
		printf("Mix_Init: Failed to init required ogg and mod support!\n");
		printf("Mix_Init: %s\n", Mix_GetError());
		return;
	}

	// open 44.1KHz, signed 16bit, system byte order,
	//      stereo audio, using 1024 byte chunks
	if(Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024)==-1) {
		printf("Mix_OpenAudio: %s\n", Mix_GetError());
		return;
	}

	Mix_AllocateChannels(16);
	mRifle = Mix_LoadWAV("share/rifle.wav");
	if(!mRifle) {
		printf("Couldn't load rifle sound!\n");
	}
}

void Sound::play(const Common::Vector3& pos)
{
	if(!mRifle)
		return;

	unsigned int dist = static_cast<unsigned int>(mCamPos.distance(pos));
	if(dist > 255)
		return;

	Common::Vector2 om(mCamLook.x, mCamLook.z);
	if(om.null())
		om.x = 1.0f;
	Common::Vector2 oq(pos.x - mCamPos.x, pos.z - mCamPos.z);
	unsigned int angle = 360 + Common::Math::radiansToDegrees(om.angleTo360(oq));
	if(angle > 360)
		angle -= 360;

	Mix_SetPosition(mNextChannel, angle, static_cast<unsigned char>(dist));
	if(Mix_PlayChannel(mNextChannel, mRifle, 0) == -1) {
		printf("Mix_PlayMusic: %s\n", Mix_GetError());
	}
	mNextChannel = (mNextChannel + 1) % 16;
}

void Sound::setCamera(const Common::Vector3& pos, const Common::Vector3& lookdir)
{
	mCamPos = pos;
	mCamLook = lookdir;
}


