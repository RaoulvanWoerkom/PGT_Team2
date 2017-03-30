#ifndef __Helper_h_
#define __Helper_h_


#include "Ogre.h"
class Helper 
{


public:
	static void log(std::string name, int number);
	static void log(std::string name, float fl);
	static void log(std::string name, long l);
	static void log(std::string name, size_t size);
	static void log(std::string name, Ogre::Vector2 vec);
	static void log(std::string name, Ogre::Vector3 vec);
	static bool vectorListContainsVector2(std::vector<Ogre::Vector2> &list, Ogre::Vector2 vec);
};
#endif