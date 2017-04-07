#ifndef __ForceGenerator_h_
#define __ForceGenerator_h_

#include "RigidBody.h"

class ForceGenerator
{
public:

	/**
	* Overload this in implementations of the interface to calculate
	* and update the force applied to the given rigid body.
	*/
	virtual void updateForce(RigidBody *body, Ogre::Real duration) = 0;
};

/**
* A force generator that applies a gravitational force. One instance
* can be used for multiple rigid bodies.
*/
class Gravity : public ForceGenerator
{
	/** Holds the acceleration due to gravity. */
	Ogre::Vector3 gravity;

public:

	/** Creates the generator with the given acceleration. */
	Gravity();

	/** Applies the gravitational force to the given rigid body. */
	virtual void updateForce(RigidBody *body, Ogre::Real duration);
};


class ForceRegistry
{
protected:

	struct ForceRegistration
	{
		RigidBody *body;
		ForceGenerator *fg;
	};

	//Holds the list of registrations.
	typedef std::vector<ForceRegistration> Registry;
	Registry registrations;

public:

	void add(RigidBody* body, ForceGenerator *fg);
	void remove(RigidBody* body, ForceGenerator *fg);
	void clear();
	void updateForces(Ogre::Real duration);
};
#endif // #ifndef __ForceGenerator_h_
