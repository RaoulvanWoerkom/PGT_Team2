#include "Ball.h"
#include "Helper.h"

void Ball::updateCameraNode()
{
	Ball::cameraNode->setPosition(Ball::getPosition());
}

Ball::Ball(Ogre::SceneNode* _node, Ogre::SceneNode* _camNode, Ogre::Entity* _entity) : RigidBody(_node, _entity)
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
	

	Helper::getMeshInformation(entity->getMesh(), vertexCount, vertices, indexCount, indices, node->getPosition(), node->getOrientation(), node->getScale());
	normals = new Ogre::Vector3[indexCount / 3];
	int index = 0;
	for (int i = 0; i < indexCount / 3; i += 3)
	{
		long index1 = indices[i];
		long index2 = indices[i + 1];
		long index3 = indices[i + 2];
		Ogre::Vector3 point1 = vertices[index1];
		Ogre::Vector3 point2 = vertices[index2];
		Ogre::Vector3 point3 = vertices[index3];
		Ogre::Vector3 currNormal = Helper::normalVector(point1, point2, point3);
		normals[index] = currNormal;
		index++;
	}
	//<----hier vult hij alle data van de mesh van de ball---->

	//maakt copy gebasseerd op data van mesh--DEZE DUS FIXEN
	//createCopy();
	//RigidBody::cut(node->getPosition(), Ogre::Vector3(0, 1, 1));
}

Ball::Ball(void)
{
}

void Ball::integrate(Ogre::Real delta)
{
	RigidBody::integrate(delta);

	updateCameraNode();
}
