#ifndef BRIGADES_RANDOM_HPP
#define BRIGADES_RANDOM_HPP

#include <array>

#include <common/Random.h>

class Random {
	public:
		enum Source {
			SourceWorld,
			SourceGame,
			SourceAI,
			SourceLast = SourceAI
		};

		static void seed(Source s, unsigned int i);
		static float clamped(Source s); // between -1 and 1
		static float uniform(Source s); // between 0 and 1
		static float uniform(Source s, float a, float b); // between a and b
		static unsigned int uniform(Source s, unsigned int i, unsigned int j); // between i and j - 1

	private:
		static std::array<Common::RandGen, SourceLast> mGens;
};

#endif

