#include "random.hpp"

std::array<Common::RandGen, Random::SourceLast> Random::mGens;

void Random::seed(Source s, unsigned int i)
{
	mGens[s] = Common::RandGen(i);
}

float Random::clamped(Source s)
{
	return mGens[s].clamped();
}

float Random::uniform(Source s)
{
	return mGens[s].uniform();
}

float Random::uniform(Source s, float a, float b)
{
	return mGens[s].uniform(a, b);
}

unsigned int Random::uniform(Source s, unsigned int i, unsigned int j)
{
	return mGens[s].uniform(i, j);
}

