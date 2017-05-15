#ifndef __CustomCamera_h_
#define __CustomCamera_h_

#include "BaseApplication.h"

class CustomCamera
{
public:
	CustomCamera(Ogre::Camera* _camera, Ogre::SceneManager* _manager, Ogre::RenderWindow* _window);
	CustomCamera(void);

	Ogre::SceneNode* camNode;
	Ogre::SceneNode* camPitchNode;
	Ogre::Camera* camera;
	Ogre::SceneManager* manager;
	Ogre::RenderWindow* window;

	void setCameraTarget(Ogre::SceneNode* node);
	void createViewports();
	void zoomCamera(float zoomAmount);
};

#endif // #ifndef __CustomCamera_h_