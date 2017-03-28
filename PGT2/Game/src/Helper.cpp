

#include "Helper.h"
#include <windows.h>
#include <stdio.h>

void Helper::log(std::string name, int number)
{ 
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, "%s: %d\n", charAr, number);
	OutputDebugString(buffer);
}

void Helper::log(std::string name, float fl)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, " %s: %f\n", charAr, fl);
	OutputDebugString(buffer);
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
	OutputDebugString(buffer);
}

//werkt niet meer???
void Helper::log(std::string name, long l)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];

	sprintf(buffer, " %s: %f\n", charAr, l);
	OutputDebugString(buffer);
}

void Helper::log(std::string name, Ogre::Vector2 vec)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	float x = vec.x;
	float y = vec.y;
	sprintf(buffer, " %s:  x: %f  y: %f\n", charAr, x, y);
	OutputDebugString(buffer);
}

void Helper::log(std::string name, Ogre::Vector3 vec)
{
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	float x = vec.x;
	float y = vec.y;
	float z = vec.z;
	sprintf(buffer, " %s:  x: %f  y: %f  z: %f\n", charAr, x, y, z);
	OutputDebugString(buffer);
}


