#include "BaseApplication.h"

class RigidBody 
{
protected:
	Ogre::Node* Node;
	Ogre::Entity* Entity;

	float InverseMass;
	Ogre::Vector3 AllForce;
	Ogre::Vector3 Torque;
	Ogre::Vector3 Velocity;
	Ogre::Vector3 Rotation;
	Ogre::Vector3 Acceleration;

	bool IsAwake;
	bool CanSleep;

public:
	void SetPosition(Ogre::Vector3 &position);
	void SetOrientation(Ogre::Quaternion &orientation);
	void AddForce(Ogre::Vector3 &force);
	void AddTorque(Ogre::Vector3 &torque);
	void SetIsAwake(const bool awake);
	void Integrate();
};

