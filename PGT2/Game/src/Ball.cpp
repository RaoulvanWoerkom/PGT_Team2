#include "Ball.h"
#include "Helper.h"
/// \brief A method to update the Camera position
///
/// A method used to update the Camera position by giving the Scenenode where the camera is attached to
/// the exact same position as the Ball object.
void Ball::updateCameraNode()
{
	Ball::cameraNode->setPosition(Ball::getPosition());
}
/// \brief A method to create a ball object from the ball class
///
/// With this method you can create a Ball object from the Ball class.
/// Here the variables needed for the Ball object are initialised and defined using variables passed on from elsewhere
/// and hardcoded variables.
/// After initialising and defining all variables the method makes a copy of the mesh used for the Ball object.
Ball::Ball(Ogre::SceneNode* _node, Ogre::SceneNode* _camNode, Ogre::Entity* _entity) : RigidBody(_node, _entity)
{
	
	Ball::node = _node;
	Ball::cameraNode = _camNode;
	Ball::entity = _entity;
	Ball:isBreakable = false;
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
/// \brief A method to update the ball object.
///
/// This method updates all forces working on the Ball object en converts them to velocity
/// it also calls the UpdateCameraNode method to make sure the camera stays on top of the Ball.
void Ball::integrate(Ogre::Real delta)
{
	RigidBody::integrate(delta);

	updateCameraNode();
}
