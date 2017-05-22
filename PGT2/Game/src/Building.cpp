#include "Building.h"


Building::Building(Ogre::SceneNode* _node, Ogre::Entity* _entity) : RigidBody(_node, _entity)
{
	Building::node = _node;
	Building::entity = _entity;
	Building::isBreakable = true;
	Building::inverseMass = 1;
	Building::dampening = 0.995;
	Building::isAwake = true;
	Building::canSleep = false;
	Building::velocity = Ogre::Vector3().ZERO;
	Building::acceleration = Ogre::Vector3(0.0f, -9.81f, 0.0f);
	Building::forceAccum = Ogre::Vector3().ZERO;
	Building::torqueAccum = Ogre::Vector3().ZERO;
	Building::rotation = Ogre::Vector3().ZERO;
	Building::inertiaTensor = Ogre::Matrix3(0.4f, 0, 0,
		0, 0.4f, 0,
		0, 0, 0.4f);
	Building::inertiaTensor.Inverse(RigidBody::inverseInertiaTensor);
}

Building::Building(void)
{
}

void Building::integrate(Ogre::Real delta)
{
	//RigidBody::integrate(delta);
}

void Building::fracture()
{
	if (!Building::isDestroyed)
	{
		srand(time(NULL));
		int randNum1 = rand() % (100);
		int randNum2 = rand() % (100);
		int randNum3 = rand() % (100);
		Ogre::Vector3 ranDir = Ogre::Vector3(0, 1, 0);
		ranDir.normalise();
		int sliceAmount = 3;
		RigidBody::cut(RigidBody::getPosition(), ranDir, sliceAmount, true);
	}
	

}
