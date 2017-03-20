#include "RigidBody.h"
#include "Helper.h"
#include <cmath>
RigidBody::RigidBody(Ogre::SceneNode* node, Ogre::Entity* entity)
{
	RigidBody::Node = node;
	RigidBody::Entity = entity;
	RigidBody::InverseMass = 1;
	RigidBody::Dampening = 0.995;
	RigidBody::IsAwake = true;
	RigidBody::CanSleep = false;
	RigidBody::Velocity = Ogre::Vector3().ZERO;
	RigidBody::Acceleration = Ogre::Vector3().ZERO;
	RigidBody::ForceAccum = Ogre::Vector3().ZERO;
}

RigidBody::RigidBody(void)
{
}

void RigidBody::SetPosition(Ogre::Vector3& position)
{
	RigidBody::Node->setPosition(position);
}

void RigidBody::SetVelocity(Ogre::Vector3 & velocity)
{
	RigidBody::Velocity = velocity;
}

Ogre::Vector3 RigidBody::GetVelocity()
{
	return RigidBody::Velocity;
}

Ogre::Vector3 RigidBody::GetPosition()
{
	return RigidBody::Node->getPosition();
}

void RigidBody::SetOrientation(Ogre::Quaternion& orientation)
{
	RigidBody::Node->setOrientation(orientation);
}

Ogre::Quaternion RigidBody::GetOrientation()
{
	return RigidBody::Node->getOrientation();
}

void RigidBody::AddForce(Ogre::Vector3& force)
{
	RigidBody::ForceAccum += force;
}

void RigidBody::AddTorque(Ogre::Vector3& torque)
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
		Ogre::Vector3 LastFrameAcceleration = RigidBody::Acceleration;
		LastFrameAcceleration += (RigidBody::ForceAccum * RigidBody::InverseMass);

		//Update velocity with time and acceleration
		RigidBody::Velocity += (LastFrameAcceleration);

		//dampen the movement so it stops eventually !Temporary fix pls watch me
		RigidBody::Velocity *= RigidBody::Dampening;

		//RigidBody::Velocity *= RigidBody::Dampening;

		//Move Rigidbody with velocity and time
		Ogre::Vector3 tempPos = RigidBody::GetPosition();
		tempPos += (RigidBody::Velocity * delta);
		RigidBody::SetPosition(tempPos);

		Helper::log("velocity", LastFrameAcceleration);
		Helper::log("acceleration", Acceleration);

		//Helper::log("acceleration", Acceleration);
		//Helper::log("forceaccum", ForceAccum);

		RigidBody::ForceAccum = Ogre::Vector3().ZERO;
		//TODO Calculate total movement and check if under benchmark: IsAwake = false
	}
}

bool RigidBody::HasFiniteMass()
{
	if (RigidBody::InverseMass == 0)
	{
		return false;
	}
	return true;
}

float RigidBody::GetMass()
{
	return 1 / RigidBody::InverseMass;
}

void RigidBody::SetInertiaTensor(const Ogre::Matrix3& inertiaTensor)
{
	RigidBody::InverseInertiaTensor = inertiaTensor.Inverse();

}

