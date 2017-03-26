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
	RigidBody::torqueAccum = Ogre::Vector3().ZERO;
	RigidBody::rotation = Ogre::Vector3().ZERO;
	RigidBody::inertiaTensor = Ogre::Matrix3(0.4f, 0, 0,
											 0, 0.4f, 0,
											 0 , 0, 0.4f );
	RigidBody::inertiaTensor.Inverse(RigidBody::inverseInertiaTensor);
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
	orientation.normalise();
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

void RigidBody::addForceAtBodyPoint(Ogre::Vector3 force, Ogre::Vector3 point)
{
	// Convert to coordinates relative to center of mass.
	Ogre::Vector3 pt = RigidBody::node->convertLocalToWorldPosition(point);
	RigidBody::addForceAtPoint(force, pt);
}

void RigidBody::addForceAtPoint(Ogre::Vector3 force, Ogre::Vector3 point)
{
	// Convert to coordinates relative to center of mass.
	Ogre::Vector3 pt = point;
	Ogre::Vector3 pos = RigidBody::getPosition();
	pt -= pos;

	RigidBody::forceAccum += force;
	RigidBody::torqueAccum += Ogre::Vector3(pt.y*force.z - pt.z*force.y,
											pt.z*force.x - pt.x*force.z,
											pt.x*force.y - pt.y*force.x);  ;
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

		Ogre::Vector3 AngularAcceleration = (RigidBody::inverseInertiaTensorWorld * RigidBody::torqueAccum);

		//Update velocity with time and acceleration
		RigidBody::velocity += (LastFrameAcceleration);
		RigidBody::rotation += (AngularAcceleration);

		//dampen the movement so it stops eventually !Temporary fix pls watch me
		RigidBody::velocity *= RigidBody::dampening;
		//RigidBody::rotation *= RigidBody::dampening;

		//Move Rigidbody with velocity and time
		Ogre::Vector3 tempPos = RigidBody::getPosition();
		tempPos += (RigidBody::velocity * delta);
		RigidBody::setPosition(tempPos);

		Ogre::Quaternion tempOrien = RigidBody::getOrientation();
		tempOrien = tempOrien + Ogre::Quaternion(0,
					RigidBody::rotation.x * delta,
					RigidBody::rotation.y * delta,
					RigidBody::rotation.z * delta);

		RigidBody::setOrientation(tempOrien);

		calculateDerivedData();



		RigidBody::forceAccum = Ogre::Vector3().ZERO;
		RigidBody::torqueAccum = Ogre::Vector3().ZERO;

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

