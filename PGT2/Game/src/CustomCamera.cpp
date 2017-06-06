#include "CustomCamera.h"
#include "BaseApplication.h"
#include "Ogre.h"


const Ogre::Vector3 START_CAMERA_DISTANCE = Ogre::Vector3(0, 250, 500);
/// \brief Constructor used to instantiates a Camera object.
///
/// This method only makes pointer from these local variables to the classes themselves.
/// I chose to use this method because we needed a simple camera that could follow the ball and rotate around it
/// luckily the Ogre::Camera class is perfect for this.
/// This constructor is only called at the beginning of the game.
CustomCamera::CustomCamera(Ogre::Camera* _camera, Ogre::SceneManager* _manager, Ogre::RenderWindow* _window)
{
	camera = _camera;
	manager = _manager;
	window = _window;
	createViewports();
}
/// \brief Empty constructor needed for C++
///
/// A constructor that is needed to make the program work, it is empty.
CustomCamera::CustomCamera(void)
{

}

/// \brief Method to create a Viewport for the CustomCamera Class.
///
/// Here we create a Viewport and attach it to the CustomCamera we created.
/// A Viewport is the meeting o a camera and a rendering surface in other words the Viewport displays what the camera sees.
/// In this method we put the BackgroundColour to black for simplicity and the aspect ratio of the Viewport to
/// the hight and with of the screen the game is played on.
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
/// This is done so that the Camera can move around the Ball object freely without following the balls rotary movements.
void CustomCamera::setCameraTarget(Ogre::SceneNode* node)
{
	camNode = node->createChildSceneNode();
	camNode->setPosition(0, 0, 0);
	camPitchNode = camNode->createChildSceneNode();
	camPitchNode->setPosition(START_CAMERA_DISTANCE);
	camPitchNode->attachObject(camera);
	camera->setAutoTracking(true, node);
}
/// \brief Method used for zooming in/out with the CustomCamera.
///
/// This method is called on mouseMovement where the mousewheel is rotated(mousewheel scrolling is also considered a mouseMovement).
/// Using the amount that the player has scrolled with his mouse(zoomAmount) we displace the camPitchNode so that it moves further or closer to the camNode.
void CustomCamera::zoomCamera(float zoomAmount)
{
	Ogre::Vector3 nodePostion = camPitchNode->getPosition();
	nodePostion.z += zoomAmount;
	camPitchNode->setPosition(nodePostion);
}



