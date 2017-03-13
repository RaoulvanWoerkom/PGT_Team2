#include "RigidBody.h"

void RigidBody::SetPosition(Ogre::Vector3 & position)
{
	RigidBody::Node->setPosition(position);
}

void RigidBody::SetOrientation(Ogre::Quaternion & orientation)
{
	RigidBody::Node->setOrientation(orientation);
}

void RigidBody::AddForce(Ogre::Vector3 & force)
{
	RigidBody::AllForce += force;
}

void RigidBody::AddTorque(Ogre::Vector3 & torque)
{
	RigidBody::Torque = torque;
}


void RigidBody::SetIsAwake(const bool awake)
{
	RigidBody::IsAwake = awake;
}

void RigidBody::Integrate()
{
	if (RigidBody::IsAwake)
	{

	}
}

