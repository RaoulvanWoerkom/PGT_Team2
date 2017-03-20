#ifndef __RigidBody_h_
#define __RigidBody_h_


#include "BaseApplication.h"

class RigidBody 
{
protected:
	

	Ogre::Real InverseMass;
	float Dampening;
	Ogre::Vector3 ForceAccum;
	Ogre::Vector3 TorqueAccum;
	Ogre::Vector3 Velocity;
	Ogre::Vector3 Rotation;
	Ogre::Vector3 Acceleration;

	Ogre::Matrix4 TransformMatrix;
	Ogre::Matrix3 InverseInertiaTensor;
	Ogre::Matrix3 InertiaTensor;


	bool IsAwake;
	bool CanSleep;

public:
	Ogre::SceneNode* Node; //contains Position & Orientation
	Ogre::Entity* Entity;

	RigidBody(Ogre::SceneNode *node, Ogre::Entity *entity);
	RigidBody(void);
	void SetPosition(Ogre::Vector3 &position);
	void SetVelocity(Ogre::Vector3 &velocity);
	Ogre::Vector3 GetVelocity();
	Ogre::Vector3 GetPosition();
	void SetOrientation(Ogre::Quaternion &orientation);
	void SetInertiaTensor(const Ogre::Matrix3 &inertiaTensor);
	Ogre::Quaternion GetOrientation();
	void AddForce(Ogre::Vector3 &force);
	void AddTorque(Ogre::Vector3 &torque);
	void SetIsAwake(const bool awake);
	void Integrate(float delta);
	bool HasFiniteMass();
	float GetMass();

};

#endif // #ifndef __RigidBody_h_