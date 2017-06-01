#include "Sound.h"
#pragma comment(lib, "irrKlang.lib") //link the library

using namespace irrklang;

Sound::Sound()
{

}

Sound::~Sound(void)
{

}

int Sound::loopingSound()
{
	ISoundEngine* engine = createIrrKlangDevice(); //instantiate an soundengine

	if (!engine) //error with the engine
	{
		return 0;
	}

	engine->play2D("media/sounds/getout.ogg", true);

	return 0;
}