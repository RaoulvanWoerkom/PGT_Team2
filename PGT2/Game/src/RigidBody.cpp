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
	RigidBody::lastFrameAcceleration = Ogre::Vector3().ZERO;
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

Ogre::Vector3 RigidBody::getRotation()
{
	return RigidBody::rotation;
}

Ogre::Vector3 RigidBody::getPosition()
{
	return RigidBody::node->getPosition();
}

Ogre::Vector3 RigidBody::getLastFrameAcceleration()
{
	return RigidBody::lastFrameAcceleration;
}

Ogre::Matrix3 RigidBody::getInverseInertiaTensorWorld()
{
	return RigidBody::inverseInertiaTensorWorld;
}

void RigidBody::addVelocity(Ogre::Vector3 vel)
{
	RigidBody::velocity += vel;
}

void RigidBody::addRotation(Ogre::Vector3 rot)
{
	RigidBody::rotation += rot;
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

void RigidBody::addForce(Ogre::Vector3 force)
{
	RigidBody::forceAccum += force;
}

void RigidBody::addTorque(Ogre::Vector3 torque)
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


void RigidBody::_transformInertiaTensor(Ogre::Matrix3 &iitWorld,
	 Ogre::Quaternion q,
	 Ogre::Matrix3 &iitBody,
	 Ogre::Matrix4 &rotmat)
{
	Ogre::Real t4 = rotmat[0][0] * iitBody[0][0] +
		rotmat[0][1] * iitBody[1][0] +
		rotmat[0][2] * iitBody[2][0];
	Ogre::Real t9 = rotmat[0][0] * iitBody[0][1] +
		rotmat[0][1] * iitBody[1][1] +
		rotmat[0][2] * iitBody[2][1];
	Ogre::Real t14 = rotmat[0][0] * iitBody[0][2] +
		rotmat[0][1] * iitBody[1][2] +
		rotmat[0][2] * iitBody[2][2];
	Ogre::Real t28 = rotmat[1][0] * iitBody[0][0] +
		rotmat[1][1] * iitBody[1][0] +
		rotmat[1][2] * iitBody[2][0];
	Ogre::Real t33 = rotmat[1][0] * iitBody[0][1] +
		rotmat[1][1] * iitBody[1][1] +
		rotmat[1][2] * iitBody[2][1];
	Ogre::Real t38 = rotmat[1][0] * iitBody[0][2] +
		rotmat[1][1] * iitBody[1][2] +
		rotmat[1][2] * iitBody[2][2];
	Ogre::Real t52 = rotmat[2][0] * iitBody[0][0] +
		rotmat[2][1] * iitBody[1][0] +
		rotmat[2][2] * iitBody[2][0];
	Ogre::Real t57 = rotmat[2][0] * iitBody[0][1] +
		rotmat[2][1] * iitBody[1][1] +
		rotmat[2][2] * iitBody[2][1];
	Ogre::Real t62 = rotmat[2][0] * iitBody[0][2] +
		rotmat[2][1] * iitBody[1][2] +
		rotmat[2][2] * iitBody[2][2];

	iitWorld[0][0] = t4*rotmat[0][0] +
		t9*rotmat[0][1] +
		t14*rotmat[0][2];
	iitWorld[0][1] = t4*rotmat[1][0] +
		t9*rotmat[1][1] +
		t14*rotmat[1][2];
	iitWorld[0][2] = t4*rotmat[2][0] +
		t9*rotmat[2][1] +
		t14*rotmat[2][2];
	iitWorld[1][0] = t28*rotmat[0][0] +
		t33*rotmat[0][1] +
		t38*rotmat[0][2];
	iitWorld[1][1] = t28*rotmat[1][0] +
		t33*rotmat[1][1] +
		t38*rotmat[1][2];
	iitWorld[1][2] = t28*rotmat[2][0] +
		t33*rotmat[2][1] +
		t38*rotmat[2][2];
	iitWorld[2][0] = t52*rotmat[0][0] +
		t57*rotmat[0][1] +
		t62*rotmat[0][2];
	iitWorld[2][1] = t52*rotmat[1][0] +
		t57*rotmat[1][1] +
		t62*rotmat[1][2];
	iitWorld[2][2] = t52*rotmat[2][0] +
		t57*rotmat[2][1] +
		t62*rotmat[2][2];
}

void RigidBody::clearAccumulators()
{
	RigidBody::forceAccum = Ogre::Vector3::ZERO;
	RigidBody::torqueAccum = Ogre::Vector3::ZERO;
}

void RigidBody::calculateDerivedData()
{
	// Calculate the transform matrix for the body.
	RigidBody::transformMatrix = RigidBody::node->_getFullTransform();

	// Calculate the inertiaTensor in world space.
	_transformInertiaTensor(RigidBody::inverseInertiaTensorWorld,
		RigidBody::getOrientation(),
		RigidBody::inverseInertiaTensor,
		RigidBody::transformMatrix);

}


void RigidBody::integrate(Ogre::Real delta)
{
	if (RigidBody::isAwake)
	{
		//calculate acceleration with mass and force
		//TODO calculate angular acceleration with Tensor and Torque
		RigidBody::lastFrameAcceleration = RigidBody::acceleration;
		RigidBody::lastFrameAcceleration += (RigidBody::forceAccum * RigidBody::inverseMass);

		Ogre::Vector3 AngularAcceleration = (RigidBody::inverseInertiaTensorWorld * RigidBody::torqueAccum);

		//Update velocity with time and acceleration
		RigidBody::velocity += (RigidBody::lastFrameAcceleration);
		RigidBody::rotation += (AngularAcceleration);

		//dampen the movement so it stops eventually !Temporary fix pls watch me
		RigidBody::velocity *= RigidBody::dampening;
		RigidBody::rotation *= RigidBody::dampening;

		//Move Rigidbody with velocity and time
		Ogre::Vector3 tempPos = RigidBody::getPosition();
		tempPos += (RigidBody::velocity * delta);
		RigidBody::setPosition(tempPos);

		Ogre::Quaternion q(0,
			RigidBody::rotation.x * delta,
			RigidBody::rotation.y * delta,
			RigidBody::rotation.z * delta);
		Ogre::Quaternion tempOrien = RigidBody::getOrientation();
		q = q * tempOrien;
		tempOrien.w += q.w * ((Ogre::Real)0.5);
		tempOrien.x += q.x * ((Ogre::Real)0.5);
		tempOrien.y += q.y * ((Ogre::Real)0.5);
		tempOrien.z += q.z * ((Ogre::Real)0.5);

		RigidBody::setOrientation(tempOrien);

		calculateDerivedData();

		clearAccumulators();

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

Ogre::Real RigidBody::getMass()
{
	return 1 / RigidBody::inverseMass;
}

Ogre::Real RigidBody::getInverseMass()
{
	return RigidBody::inverseMass;
}

void RigidBody::setInertiaTensor(const Ogre::Matrix3& inertiaTensor)
{
	RigidBody::inverseInertiaTensor = inertiaTensor.Inverse();

}

