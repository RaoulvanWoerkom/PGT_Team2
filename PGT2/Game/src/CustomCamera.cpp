#include "CustomCamera.h"
#include "BaseApplication.h"
#include "Ogre.h"

CustomCamera::CustomCamera(Ogre::Camera* _camera, Ogre::SceneManager* _manager, Ogre::RenderWindow* _window)
{
	camera = _camera;
	manager = _manager;
	window = _window;
}

CustomCamera::CustomCamera(void)
{

}


void CustomCamera::createViewports()
{
	Ogre::Viewport* vp = window->addViewport(camera);

	vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));

	camera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) /
		Ogre::Real(vp->getActualHeight()));

}

void CustomCamera::setCameraTarget(Ogre::SceneNode* node)
{
	camNode = node->createChildSceneNode();
	camNode->setPosition(0, 0, 0);
	camPitchNode = camNode->createChildSceneNode();
	camPitchNode->setPosition(0, 250, 500);
	camPitchNode->attachObject(camera);
	camera->setAutoTracking(true, node);
}

void CustomCamera::zoomCamera(float zoomAmount)
{
	Ogre::Vector3 nodePostion = camPitchNode->getPosition();
	nodePostion.z += zoomAmount;
	camPitchNode->setPosition(nodePostion);
}



