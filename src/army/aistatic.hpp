#ifndef BRIGADES_AISTATIC_HPP
#define BRIGADES_AISTATIC_HPP

#include <common/Vector3.h>

#include "worldmap.hpp"

class AIStatic {
	public:
		struct Collision {
			bool Found = false;
			Common::Vector3 Position;
			Common::Vector2 Normal;
		};

		static void setWorldMap(const WorldMap* wmap);
		static Collision getCollision(const Common::Vector3& pos, const Common::Vector3& endpos);

		class Debug {
			public:
				static void setScene(Scene::Scene* scene);
				static void drawLine(const unsigned int id, const std::string& linename,
						const Common::Vector3& start,
						const Common::Vector3& end,
						const Common::Color& col);
				static void enable();

			private:
				static Scene::Scene* mScene;
				static bool mEnabled;
		};

	private:
		static const WorldMap* mMap;
};

#endif
