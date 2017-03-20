#include "Helper.h"
#include <windows.h>
#include <stdio.h>


void Helper::log(std::string name, int number)
{ 
	char charAr[1024];
	strncpy(charAr, name.c_str(), sizeof(charAr));
	charAr[sizeof(charAr) - 1] = 0;
	char  buffer[sizeof(charAr) + 200];
	sprintf_s(buffer, "%s: %d \n", charAr, number);
	OutputDebugString(charAr);
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
