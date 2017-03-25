#include "RigidBody.h"
#include "Helper.h"
#include <cmath>
RigidBody::RigidBody(Ogre::SceneNode* _node, Ogre::Entity* _entity)
{
	RigidBody::node = _node;
	RigidBody::entity = _entity;
	RigidBody::inverseMass = 1;
	RigidBody::dampening = 0.995;
	RigidBody::isAwake = true;
	RigidBody::canSleep = false;
	RigidBody::velocity = Ogre::Vector3().ZERO;
	RigidBody::acceleration = Ogre::Vector3().ZERO;
	RigidBody::forceAccum = Ogre::Vector3().ZERO;
}

RigidBody::RigidBody(void)
{
}

void RigidBody::setPosition(Ogre::Vector3 position)
{
	RigidBody::node->setPosition(position);
}

void RigidBody::setVelocity(Ogre::Vector3 velocity)
{
	RigidBody::velocity = velocity;
}

Ogre::Vector3 RigidBody::getVelocity()
{
	return RigidBody::velocity;
}

Ogre::Vector3 RigidBody::getPosition()
{
	return RigidBody::node->getPosition();
}

void RigidBody::setOrientation(Ogre::Quaternion& orientation)
{
	RigidBody::node->setOrientation(orientation);
}

Ogre::Quaternion RigidBody::getOrientation()
{
	return RigidBody::node->getOrientation();
}

void RigidBody::addForce(Ogre::Vector3& force)
{
	RigidBody::forceAccum += force;
}

void RigidBody::addTorque(Ogre::Vector3& torque)
{
	RigidBody::torqueAccum = torque;
}


void RigidBody::setIsAwake(const bool awake)
{
	RigidBody::isAwake = awake;
}

void RigidBody::integrate(float delta)
{
	if (RigidBody::isAwake)
	{
		//calculate acceleration with mass and force
		//TODO calculate angular acceleration with Tensor and Torque
		Ogre::Vector3 LastFrameAcceleration = RigidBody::acceleration;
		LastFrameAcceleration += (RigidBody::forceAccum * RigidBody::inverseMass);

		//Update velocity with time and acceleration
		RigidBody::velocity += (LastFrameAcceleration);

		//dampen the movement so it stops eventually !Temporary fix pls watch me
		RigidBody::velocity *= RigidBody::dampening;

		//RigidBody::Velocity *= RigidBody::Dampening;

		//Move Rigidbody with velocity and time
		Ogre::Vector3 tempPos = RigidBody::getPosition();
		tempPos += (RigidBody::velocity * delta);
		RigidBody::setPosition(tempPos);


		//Helper::log("acceleration", Acceleration);
		//Helper::log("forceaccum", ForceAccum);

		RigidBody::forceAccum = Ogre::Vector3().ZERO;
		//TODO Calculate total movement and check if under benchmark: IsAwake = false
	}
}

bool RigidBody::hasFiniteMass()
{
	if (RigidBody::inverseMass == 0)
	{
		return false;
	}
	return true;
}

float RigidBody::getMass()
{
	return 1 / RigidBody::inverseMass;
}

void RigidBody::setInertiaTensor(const Ogre::Matrix3& inertiaTensor)
{
	RigidBody::inverseInertiaTensor = inertiaTensor.Inverse();

}

