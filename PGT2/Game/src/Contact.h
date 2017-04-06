#ifndef __ContactData_h_
#define __ContactData_h_


#include "BaseApplication.h"
#include "RigidBody.h"
	
class ContactResolver;

class Contact
{
		
friend class ContactResolver;

public:
	RigidBody* body[2];
	Ogre::Real friction;
	Ogre::Real restitution;
	Ogre::Vector3 contactPoint;
	Ogre::Vector3 contactNormal;
	Ogre::Real penetration;
	void setBodyData(RigidBody* one, RigidBody *two,
		Ogre::Real friction, Ogre::Real restitution);

protected:
	Ogre::Matrix3 contactToWorld;
	Ogre::Vector3 contactVelocity;
	Ogre::Real desiredDeltaVelocity;
	Ogre::Vector3 relativeContactPosition[2];
	void calculateInternals(Ogre::Real duration);
	void swapBodies();
	void matchAwakeState();
	void calculateDesiredDeltaVelocity(Ogre::Real duration);
	Ogre::Vector3 calculateLocalVelocity(unsigned bodyIndex, Ogre::Real duration);
	void calculateContactBasis();
	void applyImpulse(const Ogre::Vector3 &impulse, RigidBody *body,
		Ogre::Vector3 *velocityChange, Ogre::Vector3 *rotationChange);
	void applyVelocityChange(Ogre::Vector3 velocityChange[2],
		Ogre::Vector3 rotationChange[2]);
	void applyPositionChange(Ogre::Vector3 linearChange[2],
		Ogre::Vector3 angularChange[2],
		Ogre::Real penetration);
	Ogre::Vector3 calculateFrictionlessImpulse(Ogre::Matrix3 *inverseInertiaTensor);
	Ogre::Vector3 calculateFrictionImpulse(Ogre::Matrix3 *inverseInertiaTensor);
		
};


class ContactResolver
{
protected:
	unsigned velocityIterations;
	unsigned positionIterations;
	Ogre::Real velocityEpsilon;
	Ogre::Real positionEpsilon;

	void prepareContacts(Contact *contactArray, unsigned numContacts,
		Ogre::Real duration);

	void adjustVelocities(Contact *contactArray,
		unsigned numContacts,
		Ogre::Real duration);

	void adjustPositions(Contact *contacts,
		unsigned numContacts,
		Ogre::Real duration);

public:
		
	unsigned velocityIterationsUsed;
	unsigned positionIterationsUsed;

	ContactResolver(unsigned iterations,
		Ogre::Real velocityEpsilon = (Ogre::Real)0.01,
		Ogre::Real positionEpsilon = (Ogre::Real)0.01);
	ContactResolver(unsigned velocityIterations,
		unsigned positionIterations,
		Ogre::Real velocityEpsilon = (Ogre::Real)0.01,
		Ogre::Real positionEpsilon = (Ogre::Real)0.01);

	bool isValid()
	{
		return (velocityIterations > 0) &&
			(positionIterations > 0) &&
			(positionEpsilon >= 0.0f) &&
			(positionEpsilon >= 0.0f);
	}

	void setIterations(unsigned velocityIterations,
		unsigned positionIterations);

	void setIterations(unsigned iterations);

	void setEpsilon(Ogre::Real velocityEpsilon,
		Ogre::Real positionEpsilon);

	void resolveContacts(Contact *contactArray,
		unsigned numContacts,
		Ogre::Real duration);
private:
	bool validSettings;
		
};


class ContactGenerator
{
public:
	
	virtual unsigned addContact(Contact *contact, unsigned limit) const = 0;
};


#endif // #ifndef __ContactData_h_