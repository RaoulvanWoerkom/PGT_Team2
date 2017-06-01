#include "CustomCamera.h"
#include "BaseApplication.h"
#include "Ogre.h"
/// \brief Instantiates a Camera object.
///
/// The method is used to intantiate a CustomCamera object using the Ogre Camera class, Ogre SceneManagerClass and Ogre RenderWindow class.
/// This method only makes pointer from these local variables to the classes themselves.
CustomCamera::CustomCamera(Ogre::Camera* _camera, Ogre::SceneManager* _manager, Ogre::RenderWindow* _window)
{
	camera = _camera;
	manager = _manager;
	window = _window;
}

CustomCamera::CustomCamera(void)
{

}

/// \brief Method to create a Viewport for the CustomCamera Class.
///
/// Here we create a Viewport and attach it to the CustomCamera we created.
/// A Viewport is the meeting o a camera and a rendering surface in other words the Viewport displays what the camera sees.
void CustomCamera::createViewports()
{
	Ogre::Viewport* vp = window->addViewport(camera);

	vp->setBackgroundColour(Ogre::ColourValue(0, 0, 0));

	camera->setAspectRatio(
		Ogre::Real(vp->getActualWidth()) /
		Ogre::Real(vp->getActualHeight()));

}
/// \brief SetCameraTarget is a method used for giving the Camera a scenenode to follow.
///
/// This method makes sure that we can give the CustomCamera a target to follow.
/// We do this by creating creating a ChildSceneNode(camNode) on the node that was passed while the method was invoked(node).
/// We put the position of that created node right on top of the other one and then we give the newly created node(camNode) a ChildSceneNode(camPitchNode).
/// We then place the CustomCamera on the camPitchNode and enable autotracking of the CustomCamera so that it will always have the origal node in the middle of its sight.
void CustomCamera::setCameraTarget(Ogre::SceneNode* node)
{
	camNode = node->createChildSceneNode();
	camNode->setPosition(0, 0, 0);
	camPitchNode = camNode->createChildSceneNode();
	camPitchNode->setPosition(0, 250, 500);
	camPitchNode->attachObject(camera);
	camera->setAutoTracking(true, node);
}
/// \brief Method used for zooming in/out with the CustomCamera.
///
/// Using the amount that the player has scrolled with his mouse(zoomAmount) we displace the camPitchNode so that it moves further or closer to the camNode.
void CustomCamera::zoomCamera(float zoomAmount)
{
	Ogre::Vector3 nodePostion = camPitchNode->getPosition();
	nodePostion.z += zoomAmount;
	camPitchNode->setPosition(nodePostion);
}



