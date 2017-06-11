#ifndef __InputManager_h_
#define __InputManager_h_
#include "BaseApplication.h"

class InputManager
{
public:
	static bool wDown;
	static bool aDown;
	static bool sDown;
	static bool dDown;
	static bool yDown;
	static bool spaceDown;

	InputManager();
	void keyPressed(const OIS::KeyEvent& ke);
	void keyReleased(const OIS::KeyEvent& ke);
};

#endif // #ifndef __InputManager_h_
