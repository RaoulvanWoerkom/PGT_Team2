#include "RigidBody.h"
#include "Helper.h"

RigidBody::RigidBody(Ogre::SceneNode* node, Ogre::Entity* entity)
{
	RigidBody::Node = node;
	RigidBody::Entity = entity;
	RigidBody::InverseMass = 1;
	RigidBody::Dampening = 0.9;
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
		RigidBody::Acceleration += (RigidBody::ForceAccum * RigidBody::InverseMass);

		//Update velocity with time and acceleration
		RigidBody::Velocity += (RigidBody::Acceleration * delta);

		//dampen the movement so it stops eventually
		RigidBody::Velocity *= Ogre::Math().Pow(RigidBody::Dampening, delta);

		//Move Rigidbody with velocity and time
		Ogre::Vector3 tempPos = RigidBody::GetPosition();
		tempPos += (RigidBody::Velocity * delta);
		RigidBody::SetPosition(tempPos);

		Helper::log("velocity", Velocity);

		//TODO Calculate total movement and check if under benchmark: IsAwake = false

		RigidBody::ForceAccum = Ogre::Vector3(0, 0, 0);
	}
}

void RigidBody::calculateDerivedData()
{
	//RigidBody::TransformMatrix.makeTransform(RigidBody::GetPosition(), RigidBody::Node->getScale(), RigidBody::GetOrientation().normalise);

	
}

void RigidBody::SetInertiaTensor(const Ogre::Matrix3& inertiaTensor)
{
	RigidBody::InverseInertiaTensor = inertiaTensor.Inverse();

}

