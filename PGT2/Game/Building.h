#ifndef __Building_h_
#define __Building_h_

#include "BaseApplication.h"
#include "RigidBody.h"

class Building : public RigidBody
{
public:

	Building(Ogre::SceneNode* _node, Ogre::Entity* _entity);
	Building(void);

	void integrate(Ogre::Real delta);




};

#endif // #ifndef __Building_h_