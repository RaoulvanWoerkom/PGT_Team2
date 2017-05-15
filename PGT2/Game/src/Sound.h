#ifndef __Sound_h_
#define __Sound_h_

#include "BaseApplication.h"
#include <stdio.h>
#include <irrKlang.h>

class Sound
{
public:
	Sound();
	virtual ~Sound();
	virtual int loopingSound();

};

#endif // #ifndef __Sound_h_