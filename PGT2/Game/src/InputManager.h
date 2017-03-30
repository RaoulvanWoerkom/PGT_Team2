#ifndef __InputManager_h_
#define __InputManager_h_
#include "BaseApplication.h"

class InputManager
{
public:
	static bool iDown;
	static bool jDown;
	static bool kDown;
	static bool lDown;
	static bool yDown;

	InputManager();
	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);
};

#endif // #ifndef __InputManager_h_
