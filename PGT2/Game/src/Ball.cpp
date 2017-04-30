#include "Ball.h"
#include "Helper.h"

void Ball::updateCameraNode()
{
	Ball::cameraNode->setPosition(Ball::getPosition());
}

Ball::Ball(Ogre::SceneNode* _node, Ogre::SceneNode* _camNode, Ogre::Entity* _entity)
{
	Ball::node = _node;
	Ball::cameraNode = _camNode;
	Ball::entity = _entity;
	Ball::inverseMass = 1;
	Ball::dampening = 0.995;
	Ball::isAwake = true;
	Ball::canSleep = false;
	Ball::velocity = Ogre::Vector3().ZERO;
	Ball::acceleration = Ogre::Vector3(0.0f, -9.81f, 0.0f);
	Ball::forceAccum = Ogre::Vector3().ZERO;
	Ball::torqueAccum = Ogre::Vector3().ZERO;
	Ball::rotation = Ogre::Vector3().ZERO;
	Ball::inertiaTensor = Ogre::Matrix3(0.4f, 0, 0,
		0, 0.4f, 0,
		0, 0, 0.4f);
	Ball::inertiaTensor.Inverse(RigidBody::inverseInertiaTensor);
}

Ball::Ball(void)
{
}

void Ball::integrate(Ogre::Real delta)
{
	RigidBody::integrate(delta);

	updateCameraNode();
}
