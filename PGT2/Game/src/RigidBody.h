#ifndef __RigidBody_h_
#define __RigidBody_h_


#include "BaseApplication.h"

class RigidBody 
{
protected:
	Ogre::Real inverseMass;
	float dampening;
	Ogre::Vector3 forceAccum;
	Ogre::Vector3 torqueAccum;
	Ogre::Vector3 velocity;
	Ogre::Vector3 rotation;
	Ogre::Vector3 acceleration;

	Ogre::Matrix4 transformMatrix;
	Ogre::Matrix3 inverseInertiaTensorWorld;
	Ogre::Matrix3 inverseInertiaTensor;
	Ogre::Matrix3 inertiaTensor;


	bool isAwake;
	bool canSleep;

public:
	Ogre::SceneNode* node; //contains Position & Orientation
	Ogre::Entity* entity;

	RigidBody(Ogre::SceneNode* _node, Ogre::Entity* _entity);
	RigidBody(void);
	void setPosition(Ogre::Vector3 _position);
	void setVelocity(Ogre::Vector3 _velocity);
	Ogre::Vector3 getVelocity();
	Ogre::Vector3 getPosition();
	void setOrientation(Ogre::Quaternion& _orientation);
	void setInertiaTensor(const Ogre::Matrix3& _inertiaTensor);
	Ogre::Quaternion getOrientation();
	void addForce(Ogre::Vector3& _force);
	void addTorque(Ogre::Vector3& _torque);
	void addForceAtBodyPoint(Ogre::Vector3 force, Ogre::Vector3 point);
	void addForceAtPoint(Ogre::Vector3 force, Ogre::Vector3 point);
	void setIsAwake(const bool awake);
	void _transformInertiaTensor(Ogre::Matrix3 & iitWorld, Ogre::Quaternion & q, Ogre::Matrix3 & iitBody, Ogre::Matrix4 & rotmat);
	void calculateDerivedData();
	virtual void integrate(float delta);
	bool hasFiniteMass();
	float getMass();

};

#endif // #ifndef __RigidBody_h_