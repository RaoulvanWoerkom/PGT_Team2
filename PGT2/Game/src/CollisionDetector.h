#ifndef __CollisionDetector_h_
#define __CollisionDetector_h_

#include "Contact.h"
#include "Ball.h"

class IntersectionTests;
class CollisionDetector;

/**
* Represents a primitive to detect collisions against.
*/
class CollisionPrimitive
{
public:
	/**
	* This class exists to help the collision detector
	* and intersection routines, so they should have
	* access to its data.
	*/
	friend class IntersectionTests;
	friend class CollisionDetector;

	/**
	* The rigid body that is represented by this primitive.
	*/
	RigidBody* body;

	/**
	* The offset of this primitive from the given rigid body.
	*/
	Ogre::Matrix4 offset;

	/**
	* Calculates the internals for the primitive.
	*/
	void calculateInternals();

	/**
	* This is a convenience function to allow access to the
	* axis vectors in the transform for this primitive.
	*/
	Ogre::Vector3 getAxis(unsigned index) const
	{
		Ogre::Vector3 temp = Ogre::Vector3(transform[0][index], transform[1][index], transform[2][index]);

		return temp;
	}

	/**
	* Returns the resultant transform of the primitive, calculated from
	* the combined offset of the primitive and the transform
	* (orientation + position) of the rigid body to which it is
	* attached.
	*/
	const Ogre::Matrix4& getTransform() const
	{
		return transform;
	}


protected:
	/**
	* The resultant transform of the primitive. This is
	* calculated by combining the offset of the primitive
	* with the transform of the rigid body.
	*/
	Ogre::Matrix4 transform;
};

/**
* Represents a rigid body that can be treated as an aligned bounding
* box for collision detection.
*/
class CollisionBox : public CollisionPrimitive
{
public:
	/**
	* Holds the half-sizes of the box along each of its local axes.
	*/
	Ogre::Vector3 halfSize;
};

/**
* A wrapper class that holds fast intersection tests. These
* can be used to drive the coarse collision detection system or
* as an early out in the full collision tests below.
*/
class IntersectionTests
{
	static bool boxAndBox(
		const CollisionBox &one,
		const CollisionBox &two);
};

struct CollisionData
{
	Contact *contactArray;
	Contact *contacts;
	int contactsLeft;
	unsigned contactCount;
	Ogre::Real friction;
	Ogre::Real restitution;
	Ogre::Real tolerance;

	bool hasMoreContacts()
	{
		return contactsLeft > 0;
	}

	void reset(unsigned maxContacts)
	{
		contactsLeft = maxContacts;
		contactCount = 0;
		contacts = contactArray;
	}

	void addContacts(unsigned count)
	{
		contactsLeft -= count;
		contactCount += count;

		contacts += count;
	}
};

/**
* A wrapper class that holds the fine grained collision detection
* routines.
*
* Each of the functions has the same format: it takes the details
* of two objects, and a pointer to a contact array to fill. It
* returns the number of contacts it wrote into the array.
*/
class CollisionDetector
{
public:
	static unsigned boxAndBox(
		const CollisionBox &one,
		const CollisionBox &two,
		CollisionData *data
		);
	static unsigned boxAndPoint(
		const CollisionBox & box, 
		const Ogre::Vector3 & point, 
		CollisionData * data);
	static unsigned boxAndSphere(
		const CollisionBox & box, 
		Ball & sphere, 
		CollisionData * data);
};

#endif // #ifndef __CollisionDetector_h_
