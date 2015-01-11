#include <sstream>

#include "aistatic.hpp"

const WorldMap* AIStatic::mMap = nullptr;

void AIStatic::setWorldMap(const WorldMap* wmap)
{
	mMap = wmap;
}

AIStatic::Collision AIStatic::getCollision(const Common::Vector3& pos, const Common::Vector3& endpos)
{
	Collision coll;
	HouseWall wall;
	Common::Vector3 nearest;
	auto hit = mMap->lineBlockedByWalls(pos, endpos, &nearest, &wall);
	if(!hit)
		return coll;

	coll.Found = true;
	coll.Position = nearest;
	coll.Normal = wall.getNormal();
	return coll;
}

Scene::Scene* AIStatic::Debug::mScene = nullptr;
bool AIStatic::Debug::mEnabled = false;

void AIStatic::Debug::setScene(Scene::Scene* scene)
{
	mScene = scene;
}

void AIStatic::Debug::drawLine(const unsigned int id, const std::string& linename,
		const Common::Vector3& start,
		const Common::Vector3& end,
		const Common::Color& col)
{
	if(!mEnabled)
		return;

	std::stringstream ss;
	ss << id << " " << linename;
	std::string lname = ss.str();
	mScene->clearLine(lname);
	mScene->addLine(lname, start, end, col);
}

void AIStatic::Debug::enable()
{
	mEnabled = true;
}


