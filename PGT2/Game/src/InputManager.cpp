#include "InputManager.h"

bool InputManager::iDown = false;
bool InputManager::jDown = false;
bool InputManager::kDown = false;
bool InputManager::lDown = false;
bool InputManager::yDown = false;
bool InputManager::spaceDown = false;

InputManager::InputManager()
{

}

void InputManager::keyPressed(const OIS::KeyEvent& ke)
{
	switch (ke.key)
	{
		case OIS::KC_I:
			InputManager::iDown = true;
			break;
		case OIS::KC_J:
			InputManager::jDown = true;
			break;
		case OIS::KC_K:
			InputManager::kDown = true;
			break;
		case OIS::KC_L:
			InputManager::lDown = true;
			break;
		case OIS::KC_Y:
			InputManager::yDown = true;
			break;
		case OIS::KC_SPACE:
			InputManager::spaceDown = true;
			break;
		default:
			break;
	}
}

void InputManager::keyReleased(const OIS::KeyEvent& ke)
{
	switch (ke.key)
	{
		case OIS::KC_I:
			InputManager::iDown = false;
			break;
		case OIS::KC_J:
			InputManager::jDown = false;
			break;
		case OIS::KC_K:
			InputManager::kDown = false;
			break;
		case OIS::KC_L:
			InputManager::lDown = false;
			break;
		case OIS::KC_Y:
			InputManager::yDown = false;
			break;
		case OIS::KC_SPACE:
			InputManager::spaceDown = false;
			break;
		default:
			break;
	}
}
