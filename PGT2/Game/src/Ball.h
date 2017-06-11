#ifndef __Ball_h_
#define __Ball_h_

#include "BaseApplication.h"
#include "RigidBody.h"

class Ball : public RigidBody
{
public:
	void updateCameraNode();
	Ogre::SceneNode* cameraNode;

	Ball(Ogre::SceneNode* _node, Ogre::SceneNode* _camNode, Ogre::Entity* _entity);
	Ball(void);

	void integrate(Ogre::Real delta);
	bool isGrounded;

	
	
};

#endif // #ifndef __Ball_h_