#include "Ogre.h"
class Helper 
{


public:
	static void log(std::string name, int number);
	static void log(std::string name, float fl);
	static void log(std::string name, Ogre::Vector3 vec);
	static void log(std::string name, Ogre::Quaternion qaut);
};
