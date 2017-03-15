#include "RigidBody.h"

void RigidBody::SetPosition(Ogre::Vector3 & position)
{
	RigidBody::Node->setPosition(position);
}

Ogre::Vector3 RigidBody::GetPosition()
{
	return RigidBody::Node->getPosition();
}

void RigidBody::SetOrientation(Ogre::Quaternion & orientation)
{
	RigidBody::Node->setOrientation(orientation);
}

Ogre::Quaternion RigidBody::GetOrientation()
{
	return RigidBody::Node->getOrientation();
}

void RigidBody::AddForce(Ogre::Vector3 & force)
{
	RigidBody::ForceAccum += force;
}

void RigidBody::AddTorque(Ogre::Vector3 & torque)
{
	RigidBody::TorqueAccum = torque;
}


void RigidBody::SetIsAwake(const bool awake)
{
	RigidBody::IsAwake = awake;
}

void RigidBody::Integrate(float delta)
{
	if (RigidBody::IsAwake)
	{
		//calculate acceleration with mass and force
		//TODO calculate angular acceleration with Tensor and Torque
		RigidBody::Acceleration + (RigidBody::ForceAccum * RigidBody::InverseMass);

		//Update velocity with time and acceleration
		RigidBody::Velocity + (RigidBody::Acceleration * delta);

		//dampen the movement so it stops eventually
		RigidBody::Velocity *= Ogre::Math().Pow(RigidBody::Dampening, delta);

		//Move Rigidbody with velocity and time
		Ogre::Vector3 tempPos = RigidBody::GetPosition();
		tempPos + (RigidBody::Velocity * delta);
		RigidBody::SetPosition(tempPos);

		//TODO Calculate total movement and check if under benchmark: IsAwake = false
	}
}

