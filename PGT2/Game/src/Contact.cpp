#include "Contact.h"
#include "RigidBody.h"


void Contact::setBodyData(RigidBody* one, RigidBody *two,
	Ogre::Real friction, Ogre::Real restitution)
{
	Contact::body[0] = one;
	Contact::body[1] = two;
	Contact::friction = friction;
	Contact::restitution = restitution;
}


void Contact::swapBodies()
{
	contactNormal *= -1;

	RigidBody *temp = body[0];
	body[0] = body[1];
	body[1] = temp;
}


void Contact::calculateContactBasis()
{
	Ogre::Vector3 contactTangent[2];

	if (abs(contactNormal.x) > abs(contactNormal.y))
	{
		const Ogre::Real s = (Ogre::Real)1.0f / sqrt(contactNormal.z*contactNormal.z +
			contactNormal.x*contactNormal.x);

		contactTangent[0].x = contactNormal.z*s;
		contactTangent[0].y = 0;
		contactTangent[0].z = -contactNormal.x*s;

		contactTangent[1].x = contactNormal.y*contactTangent[0].x;
		contactTangent[1].y = contactNormal.z*contactTangent[0].x -
			contactNormal.x*contactTangent[0].z;
		contactTangent[1].z = -contactNormal.y*contactTangent[0].x;
	}
	else
	{
		const Ogre::Real s = (Ogre::Real)1.0 / sqrt(contactNormal.z*contactNormal.z +
			contactNormal.y*contactNormal.y);

		contactTangent[0].x = 0;
		contactTangent[0].y = -contactNormal.z*s;
		contactTangent[0].z = contactNormal.y*s;

		contactTangent[1].x = contactNormal.y*contactTangent[0].z -
			contactNormal.z*contactTangent[0].y;
		contactTangent[1].y = -contactNormal.x*contactTangent[0].z;
		contactTangent[1].z = contactNormal.x*contactTangent[0].y;
	}

	contactToWorld.FromAxes(
		contactNormal,
		contactTangent[0],
		contactTangent[1]);
}

Ogre::Vector3 Contact::calculateLocalVelocity(unsigned bodyIndex, Ogre::Real duration)
{
	RigidBody *thisBody = body[bodyIndex];

	Ogre::Vector3 velocity =
		thisBody->getRotation().crossProduct(relativeContactPosition[bodyIndex]);
	velocity += thisBody->getVelocity();

	Ogre::Vector3 contactVelocity = Ogre::Vector3(
		velocity.x * contactToWorld[0][0] + velocity.y * contactToWorld[1][0] + velocity.z * contactToWorld[3][0],
		velocity.x * contactToWorld[0][1] + velocity.y * contactToWorld[1][1] + velocity.z * contactToWorld[3][1],
		velocity.x * contactToWorld[0][2] + velocity.y * contactToWorld[1][2] + velocity.z * contactToWorld[3][2]); // transformTranspose


	Ogre::Vector3 accVelocity = thisBody->getLastFrameAcceleration() * duration;

	accVelocity = Ogre::Vector3(
		accVelocity.x * contactToWorld[0][0] + accVelocity.y * contactToWorld[1][0] + accVelocity.z * contactToWorld[3][0],
		accVelocity.x * contactToWorld[0][1] + accVelocity.y * contactToWorld[1][1] + accVelocity.z * contactToWorld[3][1],
		accVelocity.x * contactToWorld[0][2] + accVelocity.y * contactToWorld[1][2] + accVelocity.z * contactToWorld[3][2]); // transformTranspose
	accVelocity.x = 0;

	contactVelocity += accVelocity;

	return contactVelocity;
}


void Contact::calculateDesiredDeltaVelocity(Ogre::Real duration)
{
	const static Ogre::Real velocityLimit = (Ogre::Real)0.25f;

	// Calculate the acceleration induced velocity accumulated this frame
	Ogre::Real velocityFromAcc = 0;
	
	velocityFromAcc +=
		(body[0]->getLastFrameAcceleration() * duration * contactNormal).length();
	
	if(body[1])
	velocityFromAcc -=
		(body[1]->getLastFrameAcceleration() * duration * contactNormal).length();

	// If the velocity is very slow, limit the restitution
	Ogre::Real thisRestitution = restitution;
	if (abs(contactVelocity.x) < velocityLimit)
	{
		thisRestitution = (Ogre::Real)0.0f;
	}

	// Combine the bounce velocity with the removed
	// acceleration velocity.
	desiredDeltaVelocity =
		-contactVelocity.x
		- thisRestitution * (contactVelocity.x - velocityFromAcc);
}


void Contact::calculateInternals(Ogre::Real duration)
{
	// Check if the first object is NULL, and swap if it is.
	if (!body[0]) swapBodies();
	assert(body[0]);

	// Calculate an set of axis at the contact point.
	calculateContactBasis();

	// Store the relative position of the contact relative to each body
	relativeContactPosition[0] = contactPoint - body[0]->getPosition();
	if (body[1]) {
		relativeContactPosition[1] = contactPoint - body[1]->getPosition();
	}

	// Find the relative velocity of the bodies at the contact point.
	contactVelocity = calculateLocalVelocity(0, duration);
	if (body[1]) {
		contactVelocity -= calculateLocalVelocity(1, duration);
	}

	// Calculate the desired change in velocity for resolution
	calculateDesiredDeltaVelocity(duration);
}

void Contact::applyVelocityChange(Ogre::Vector3 velocityChange[2],
	Ogre::Vector3 rotationChange[2])
{
	// Get hold of the inverse mass and inverse inertia tensor, both in
	// world coordinates.
	Ogre::Matrix3 inverseInertiaTensor[2];
	inverseInertiaTensor[0] = body[0]->getInverseInertiaTensorWorld();
	if (body[1])
		inverseInertiaTensor[1] = body[1]->getInverseInertiaTensorWorld();

	// We will calculate the impulse for each contact axis
	Ogre::Vector3 impulseContact;

	if (friction == (Ogre::Real)0.0)
	{
		// Use the short format for frictionless contacts
		impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
	}
	else
	{
		// Otherwise we may have impulses that aren't in the direction of the
		// contact, so we need the more complex version.
		impulseContact = calculateFrictionImpulse(inverseInertiaTensor);
	}

	// Convert impulse to world coordinates
	Ogre::Vector3 impulse = contactToWorld * (impulseContact);

	// Split in the impulse into linear and rotational components
	Ogre::Vector3 impulsiveTorque = relativeContactPosition[0].crossProduct(impulse);
	rotationChange[0] = inverseInertiaTensor[0] * (impulsiveTorque);
	velocityChange[0] = Ogre::Vector3::ZERO;
	velocityChange[0] += (impulse *  body[0]->getInverseMass());
	

	// Apply the changes
	body[0]->addVelocity(velocityChange[0]);
	body[0]->addRotation(rotationChange[0]);

	if (body[1])
	{
		// Work out body one's linear and angular changes
		Ogre::Vector3 impulsiveTorque = impulse.crossProduct(relativeContactPosition[1]);
		rotationChange[1] = inverseInertiaTensor[1] * (impulsiveTorque);
		velocityChange[1] = Ogre::Vector3::ZERO;
		velocityChange[1] += (impulse * -body[1]->getInverseMass());

		// And apply them.
		body[1]->addVelocity(velocityChange[1]);
		body[1]->addRotation(rotationChange[1]);
	}
}

inline
Ogre::Vector3 Contact::calculateFrictionlessImpulse(Ogre::Matrix3 * inverseInertiaTensor)
{
	Ogre::Vector3 impulseContact;

	// Build a vector that shows the change in velocity in
	// world space for a unit impulse in the direction of the contact
	// normal.
	Ogre::Vector3 deltaVelWorld = relativeContactPosition[0].crossProduct(contactNormal);
	deltaVelWorld = inverseInertiaTensor[0] * (deltaVelWorld);
	deltaVelWorld = deltaVelWorld.crossProduct(relativeContactPosition[0]);

	// Work out the change in velocity in contact coordiantes.
	Ogre::Real deltaVelocity = (deltaVelWorld * contactNormal).length();

	// Add the linear component of velocity change
	deltaVelocity += body[0]->getInverseMass();

	// Check if we need to the second body's data
	if (body[1])
	{
		// Go through the same transformation sequence again
		Ogre::Vector3 deltaVelWorld = relativeContactPosition[1].crossProduct(contactNormal);
		deltaVelWorld = inverseInertiaTensor[1] * (deltaVelWorld);
		deltaVelWorld = deltaVelWorld.crossProduct(relativeContactPosition[1]);

		// Add the change in velocity due to rotation
		deltaVelocity += (deltaVelWorld * contactNormal).length();

		// Add the change in velocity due to linear motion
		deltaVelocity += body[1]->getInverseMass();
	}

	// Calculate the required size of the impulse
	impulseContact.x = desiredDeltaVelocity / deltaVelocity;
	impulseContact.y = 0;
	impulseContact.z = 0;
	return impulseContact;
}


inline
Ogre::Vector3 Contact::calculateFrictionImpulse(Ogre::Matrix3 * inverseInertiaTensor)
{
	Ogre::Vector3 impulseContact;
	Ogre::Real inverseMass = body[0]->getInverseMass();

	// The equivalent of a cross product in matrices is multiplication
	// by a skew symmetric matrix - we build the matrix for converting
	// between linear and angular quantities.
	
	Ogre::Matrix3 impulseToTorque;
	impulseToTorque[0][0] = impulseToTorque[1][1] = impulseToTorque[2][2] = 0; //setSkewSymmertic
	impulseToTorque[0][1] = -(relativeContactPosition[0].z);
	impulseToTorque[0][2] = (relativeContactPosition[0].y);
	impulseToTorque[1][0] = (relativeContactPosition[0].z);
	impulseToTorque[1][2] = -(relativeContactPosition[0].x);
	impulseToTorque[2][0] = -(relativeContactPosition[0].y);
	impulseToTorque[2][1] = (relativeContactPosition[0].x);

	// Build the matrix to convert contact impulse to change in velocity
	// in world coordinates.
	Ogre::Matrix3 deltaVelWorld = impulseToTorque;
	deltaVelWorld = deltaVelWorld * inverseInertiaTensor[0];
	deltaVelWorld = deltaVelWorld * impulseToTorque;
	deltaVelWorld = deltaVelWorld * -1;

	// Check if we need to add body two's data
	if (body[1])
	{
		// Set the cross product matrix
		impulseToTorque[0][0] = impulseToTorque[1][1] = impulseToTorque[2][2] = 0; //setSkewSymmertic
		impulseToTorque[0][1] = -(relativeContactPosition[1].z);
		impulseToTorque[0][2] = (relativeContactPosition[1].y);
		impulseToTorque[1][0] = (relativeContactPosition[1].z);
		impulseToTorque[1][2] = -(relativeContactPosition[1].x);
		impulseToTorque[2][0] = -(relativeContactPosition[1].y);
		impulseToTorque[2][1] = (relativeContactPosition[1].x);

		// Calculate the velocity change matrix
		Ogre::Matrix3 deltaVelWorld2 = impulseToTorque;
		deltaVelWorld2 = deltaVelWorld2 * inverseInertiaTensor[1];
		deltaVelWorld2 = deltaVelWorld2 * impulseToTorque;
		deltaVelWorld2 = deltaVelWorld2 * -1;

		// Add to the total delta velocity.
		deltaVelWorld = deltaVelWorld + deltaVelWorld2;

		// Add to the inverse mass
		inverseMass += body[1]->getInverseMass();
	}

	// Do a change of basis to convert into contact coordinates.
	Ogre::Matrix3 deltaVelocity = contactToWorld.Transpose();
	deltaVelocity = deltaVelocity * deltaVelWorld;
	deltaVelocity = deltaVelocity * contactToWorld;

	// Add in the linear velocity change
	deltaVelocity[0][0] += inverseMass;
	deltaVelocity[1][1] += inverseMass;
	deltaVelocity[2][2] += inverseMass;

	// Invert to get the impulse needed per unit velocity
	Ogre::Matrix3 impulseMatrix = deltaVelocity.Inverse();

	// Find the target velocities to kill
	Ogre::Vector3 velKill(desiredDeltaVelocity,
		-contactVelocity.y,
		-contactVelocity.z);

	// Find the impulse to kill target velocities
	impulseContact = impulseMatrix * (velKill);

	// Check for exceeding friction
	Ogre::Real planarImpulse = sqrt(
		impulseContact.y*impulseContact.y +
		impulseContact.z*impulseContact.z
	);
	if (planarImpulse > impulseContact.x * friction)
	{
		// We need to use dynamic friction
		impulseContact.y /= planarImpulse;
		impulseContact.z /= planarImpulse;

		impulseContact.x = deltaVelocity[0][0] +
			deltaVelocity[0][1] * friction*impulseContact.y +
			deltaVelocity[0][2] * friction*impulseContact.z;
		impulseContact.x = desiredDeltaVelocity / impulseContact.x;
		impulseContact.y *= friction * impulseContact.x;
		impulseContact.z *= friction * impulseContact.x;
	}
	return impulseContact;
}

void Contact::applyPositionChange(Ogre::Vector3 linearChange[2],
	Ogre::Vector3 angularChange[2],
	Ogre::Real penetration)
{
	const Ogre::Real angularLimit = (Ogre::Real)0.2f;
	Ogre::Real angularMove[2];
	Ogre::Real linearMove[2];

	Ogre::Real totalInertia = 0;
	Ogre::Real linearInertia[2];
	Ogre::Real angularInertia[2];

	// We need to work out the inertia of each object in the direction
	// of the contact normal, due to angular inertia only.
	for (unsigned i = 0; i < 2; i++) if (body[i])
	{
		Ogre::Matrix3 inverseInertiaTensor;
		inverseInertiaTensor = body[i]->getInverseInertiaTensorWorld();

		// Use the same procedure as for calculating frictionless
		// velocity change to work out the angular inertia.
		Ogre::Vector3 angularInertiaWorld =
			relativeContactPosition[i].crossProduct(contactNormal);
		angularInertiaWorld =
			inverseInertiaTensor * (angularInertiaWorld);
		angularInertiaWorld =
			angularInertiaWorld.crossProduct(relativeContactPosition[i]);
		angularInertia[i] =
			(angularInertiaWorld * contactNormal).length();

		// The linear component is simply the inverse mass
		linearInertia[i] = body[i]->getInverseMass();

		// Keep track of the total inertia from all components
		totalInertia += linearInertia[i] + angularInertia[i];

		// We break the loop here so that the totalInertia value is
		// completely calculated (by both iterations) before
		// continuing.
	}

	// Loop through again calculating and applying the changes
	for (unsigned i = 0; i < 2; i++) if (body[i])
	{
		// The linear and angular movements required are in proportion to
		// the two inverse inertias.
		Ogre::Real sign = (i == 0) ? 1 : -1;
		angularMove[i] =
			sign * penetration * (angularInertia[i] / totalInertia);
		linearMove[i] =
			sign * penetration * (linearInertia[i] / totalInertia);

		// To avoid angular projections that are too great (when mass is large
		// but inertia tensor is small) limit the angular move.
		Ogre::Vector3 projection = relativeContactPosition[i];
		projection += contactNormal * -relativeContactPosition[i].dotProduct(contactNormal);

		// Use the small angle approximation for the sine of the angle (i.e.
		// the magnitude would be sine(angularLimit) * projection.magnitude
		// but we approximate sine(angularLimit) to angularLimit).
		Ogre::Real maxMagnitude = angularLimit * projection.length();

		if (angularMove[i] < -maxMagnitude)
		{
			Ogre::Real totalMove = angularMove[i] + linearMove[i];
			angularMove[i] = -maxMagnitude;
			linearMove[i] = totalMove - angularMove[i];
		}
		else if (angularMove[i] > maxMagnitude)
		{
			Ogre::Real totalMove = angularMove[i] + linearMove[i];
			angularMove[i] = maxMagnitude;
			linearMove[i] = totalMove - angularMove[i];
		}

		// We have the linear amount of movement required by turning
		// the rigid body (in angularMove[i]). We now need to
		// calculate the desired rotation to achieve that.
		if (angularMove[i] == 0)
		{
			// Easy case - no angular movement means no rotation.
			angularChange[i] = Ogre::Vector3::ZERO;
		}
		else
		{
			// Work out the direction we'd like to rotate in.
			Ogre::Vector3 targetAngularDirection =
				relativeContactPosition[i].crossProduct(contactNormal);

			Ogre::Matrix3 inverseInertiaTensor;
			inverseInertiaTensor = body[i]->getInverseInertiaTensorWorld();

			// Work out the direction we'd need to rotate to achieve that
			angularChange[i] =
				inverseInertiaTensor * (targetAngularDirection) *
				(angularMove[i] / angularInertia[i]);
		}

		// Velocity change is easier - it is just the linear movement
		// along the contact normal.
		linearChange[i] = contactNormal * linearMove[i];

		// Now we can start to apply the values we've calculated.
		// Apply the linear movement
		Ogre::Vector3 pos;
		pos = body[i]->getPosition();
		pos +=(contactNormal * linearMove[i]);
		body[i]->setPosition(pos);

		// And the change in orientation
		Ogre::Quaternion q(0,
			angularChange[i].x * ((Ogre::Real)1.0),
			angularChange[i].y * ((Ogre::Real)1.0),
			angularChange[i].z * ((Ogre::Real)1.0));
		Ogre::Quaternion tempOrien = body[i]->getOrientation();
		q = q * tempOrien;
		tempOrien.w += q.w * ((Ogre::Real)0.5);
		tempOrien.x += q.x * ((Ogre::Real)0.5);
		tempOrien.y += q.y * ((Ogre::Real)0.5);
		tempOrien.z += q.z * ((Ogre::Real)0.5);

		body[i]->setOrientation(tempOrien);

		// We need to calculate the derived data for any body that is
		// asleep, so that the changes are reflected in the object's
		// data. Otherwise the resolution will not change the position
		// of the object, and the next collision detection round will
		// have the same penetration.
		body[i]->calculateDerivedData();
	}
}





// Contact resolver implementation

ContactResolver::ContactResolver(unsigned iterations,
	Ogre::Real velocityEpsilon,
	Ogre::Real positionEpsilon)
{
	setIterations(iterations, iterations);
	setEpsilon(velocityEpsilon, positionEpsilon);
}

ContactResolver::ContactResolver(unsigned velocityIterations,
	unsigned positionIterations,
	Ogre::Real velocityEpsilon,
	Ogre::Real positionEpsilon)
{
	setIterations(velocityIterations);
	setEpsilon(velocityEpsilon, positionEpsilon);
}

void ContactResolver::setIterations(unsigned iterations)
{
	setIterations(iterations, iterations);
}

void ContactResolver::setIterations(unsigned velocityIterations,
	unsigned positionIterations)
{
	ContactResolver::velocityIterations = velocityIterations;
	ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::setEpsilon(Ogre::Real velocityEpsilon,
	Ogre::Real positionEpsilon)
{
	ContactResolver::velocityEpsilon = velocityEpsilon;
	ContactResolver::positionEpsilon = positionEpsilon;
}

void ContactResolver::resolveContacts(Contact *contacts,
	unsigned numContacts,
	Ogre::Real duration)
{
	// Make sure we have something to do.
	if (numContacts == 0) return;
	if (!isValid()) return;

	// Prepare the contacts for processing
	prepareContacts(contacts, numContacts, duration);

	// Resolve the interpenetration problems with the contacts.
	adjustPositions(contacts, numContacts, duration);

	// Resolve the velocity problems with the contacts.
	adjustVelocities(contacts, numContacts, duration);
}

void ContactResolver::prepareContacts(Contact* contacts,
	unsigned numContacts,
	Ogre::Real duration)
{
	// Generate contact velocity and axis information.
	Contact* lastContact = contacts + numContacts;
	for (Contact* contact = contacts; contact < lastContact; contact++)
	{
		// Calculate the internal contact data (inertia, basis, etc).
		contact->calculateInternals(duration);
	}
}

void ContactResolver::adjustVelocities(Contact *c,
	unsigned numContacts,
	Ogre::Real duration)
{
	Ogre::Vector3 velocityChange[2], rotationChange[2];
	Ogre::Vector3 deltaVel;

	// iteratively handle impacts in order of severity.
	velocityIterationsUsed = 0;
	while (velocityIterationsUsed < velocityIterations)
	{
		// Find contact with maximum magnitude of probable velocity change.
		Ogre::Real max = velocityEpsilon;
		unsigned index = numContacts;
		for (unsigned i = 0; i < numContacts; i++)
		{
			if (c[i].desiredDeltaVelocity > max)
			{
				max = c[i].desiredDeltaVelocity;
				index = i;
			}
		}
		if (index == numContacts) break;

		// Match the awake state at the contact!!

		// Do the resolution on the contact that came out top.
		c[index].applyVelocityChange(velocityChange, rotationChange);

		// With the change in velocity of the two bodies, the update of
		// contact velocities means that some of the relative closing
		// velocities need recomputing.
		for (unsigned i = 0; i < numContacts; i++)
		{
			// Check each body in the contact
			for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
			{
				// Check for a match with each body in the newly
				// resolved contact
				for (unsigned d = 0; d < 2; d++)
				{
					if (c[i].body[b] == c[index].body[d])
					{
						deltaVel = velocityChange[d] +
							rotationChange[d].crossProduct(
								c[i].relativeContactPosition[b]);

						// The sign of the change is negative if we're dealing
						// with the second body in a contact.
						c[i].contactVelocity +=
							Ogre::Vector3(
								deltaVel.x * c[i].contactToWorld[0][0] + deltaVel.y * c[i].contactToWorld[1][0] + deltaVel.z * c[i].contactToWorld[3][0],
								deltaVel.x * c[i].contactToWorld[0][1] + deltaVel.y * c[i].contactToWorld[1][1] + deltaVel.z * c[i].contactToWorld[3][1],
								deltaVel.x * c[i].contactToWorld[0][2] + deltaVel.y * c[i].contactToWorld[1][2] + deltaVel.z * c[i].contactToWorld[3][2]) // transformTranspose
							* (b ? -1 : 1);

						

						c[i].calculateDesiredDeltaVelocity(duration);
					}
				}
			}
		}
		velocityIterationsUsed++;
	}
}

void ContactResolver::adjustPositions(Contact *c,
	unsigned numContacts,
	Ogre::Real duration)
{
	unsigned i, index;
	Ogre::Vector3 linearChange[2], angularChange[2];
	Ogre::Real max;
	Ogre::Vector3 deltaPosition;

	// iteratively resolve interpenetrations in order of severity.
	positionIterationsUsed = 0;
	while (positionIterationsUsed < positionIterations)
	{
		// Find biggest penetration
		max = positionEpsilon;
		index = numContacts;
		for (i = 0; i<numContacts; i++)
		{
			if (c[i].penetration > max)
			{
				max = c[i].penetration;
				index = i;
			}
		}
		if (index == numContacts) break;

		// Match the awake state at the contact!!

		// Resolve the penetration.
		c[index].applyPositionChange(
			linearChange,
			angularChange,
			max);

		// Again this action may have changed the penetration of other
		// bodies, so we update contacts.
		for (i = 0; i < numContacts; i++)
		{
			// Check each body in the contact
			for (unsigned b = 0; b < 2; b++) if (c[i].body[b])
			{
				// Check for a match with each body in the newly
				// resolved contact
				for (unsigned d = 0; d < 2; d++)
				{
					if (c[i].body[b] == c[index].body[d])
					{
						deltaPosition = linearChange[d] +
							angularChange[d].crossProduct(
								c[i].relativeContactPosition[b]);

						// The sign of the change is positive if we're
						// dealing with the second body in a contact
						// and negative otherwise (because we're
						// subtracting the resolution)..
						c[i].penetration +=
							deltaPosition.dotProduct(c[i].contactNormal)
							* (b ? 1 : -1);
					}
				}
			}
		}
		positionIterationsUsed++;
	}
}
