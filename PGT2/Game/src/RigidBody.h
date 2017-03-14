#include "BaseApplication.h"

class RigidBody 
{
protected:
	Ogre::Node* Node;
	Ogre::Entity* Entity;

	float InverseMass;
	float Dampening;
	Ogre::Vector3 ForceAccum;
	Ogre::Vector3 TorqueAccum;
	Ogre::Vector3 Velocity;
	Ogre::Vector3 Rotation;
	Ogre::Vector3 Acceleration;

	bool IsAwake;
	bool CanSleep;

public:
	void SetPosition(Ogre::Vector3 &position);
	Ogre::Vector3 GetPosition();
	void SetOrientation(Ogre::Quaternion &orientation);
	Ogre::Quaternion GetOrientation();
	void AddForce(Ogre::Vector3 &force);
	void AddTorque(Ogre::Vector3 &torque);
	void SetIsAwake(const bool awake);
	void Integrate(float delta);
};

