

#include "Helper.h"
#include <stdio.h>

void Helper::log(std::string name, int number)
{ 
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, "%s: %d\n", charAr, number);
}

void Helper::log(std::string name, Ogre::Real fl)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, " %s: %f\n", charAr, fl);
}

//werkt neit
void Helper::log(std::string name, size_t size)
{
	int number = static_cast<int>(size);
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, " %s: %f\n", charAr, number);
}

//werkt niet meer???
void Helper::log(std::string name, long l)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, " %s: %f\n", charAr, l);
}

void Helper::log(std::string name, Ogre::Vector2 vec)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	Ogre::Real x = vec.x;
	Ogre::Real y = vec.y;
	sprintf(buffer, " %s:  x: %f  y: %f\n", charAr, x, y);
}

void Helper::log(std::string name, Ogre::Vector3 vec)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	Ogre::Real x = vec.x;
	Ogre::Real y = vec.y;
	Ogre::Real z = vec.z;
	sprintf(buffer, " %s:  x: %f  y: %f  z: %f\n", charAr, x, y, z);
}

void Helper::log(std::string name, Ogre::Quaternion qaut)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	Ogre::Real x = qaut.x;
	Ogre::Real y = qaut.y;
	Ogre::Real z = qaut.z;
	Ogre::Real w = qaut.w;
	sprintf(buffer, " %s:  x: %f  y: %f  z: %f, w: %f\n", charAr, x, y, z, w);
}

bool Helper::vectorListContainsVector2(std::vector<Ogre::Vector2> &list, Ogre::Vector2 vec)
{
	for (size_t i = 0; i < list.size(); i++)
	{
		Ogre::Vector2 currVec = list.at(i);
		if (currVec.x == vec.x && currVec.y == vec.y)
		{
			return true;
		}
	}
	return false;
}

