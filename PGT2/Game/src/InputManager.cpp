#include "InputManager.h"

bool InputManager::wDown = false;
bool InputManager::aDown = false;
bool InputManager::sDown = false;
bool InputManager::dDown = false;
bool InputManager::yDown = false;
bool InputManager::spaceDown = false;

InputManager::InputManager()
{

}

void InputManager::keyPressed(const OIS::KeyEvent& ke)
{
	switch (ke.key)
	{
		case OIS::KC_W:
			InputManager::wDown = true;
			break;
		case OIS::KC_A:
			InputManager::aDown = true;
			break;
		case OIS::KC_S:
			InputManager::sDown = true;
			break;
		case OIS::KC_D:
			InputManager::dDown = true;
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
		case OIS::KC_W:
			InputManager::wDown = false;
			break;
		case OIS::KC_A:
			InputManager::aDown = false;
			break;
		case OIS::KC_S:
			InputManager::sDown = false;
			break;
		case OIS::KC_D:
			InputManager::dDown = false;
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
