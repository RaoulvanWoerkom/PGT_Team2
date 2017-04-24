#ifndef __Helper_h_
#define __Helper_h_


#include "Ogre.h"
class Helper 
{


public:
	static void log(std::string name, int number);
	static void log(std::string name, Ogre::Real fl);
	static void log(std::string name, long l);
	static void log(std::string name, size_t size);
	static void log(std::string name, Ogre::Vector2 vec);
	static void log(std::string name, Ogre::Vector3 vec);
	static bool vectorListContainsVector2(std::vector<Ogre::Vector2> &list, Ogre::Vector2 vec);
	static void log(std::string name, Ogre::Quaternion qaut);
	static void getMeshInformation(const Ogre::MeshPtr mesh,
		size_t &vertex_count,
		Ogre::Vector3* &vertices,
		size_t &index_count,
		unsigned long* &indices,
		const Ogre::Vector3 &position,
		const Ogre::Quaternion &orient,
		const Ogre::Vector3 &scale);

	static Ogre::Vector3 normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3);
};
#endif
