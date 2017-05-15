#ifndef __MeshGenerator_h_
#define __MeshGenerator_h_


#include "BaseApplication.h"
#include "RigidBody.h"

class MeshGenerator : public RigidBody
{
protected:


public:
	MeshGenerator(void);
	MeshGenerator(Ogre::SceneNode* sceneNode);

	void createColourCube();

};

#endif // #ifndef __MeshGenerator_h_