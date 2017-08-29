
#include "BaseApplication.h"
#include "RigidBody.h"
#include "ParticleSystemBase.h"

class Building : public RigidBody
{

public:
	Building(Ogre::SceneNode* _node, Ogre::Entity* _entity);
	Building(void);

	void integrate(Ogre::Real delta);


	void fracture(RigidBody *ballBody);
	Ogre::SceneNode* getNode();
};
