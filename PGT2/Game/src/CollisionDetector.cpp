#include "CollisionDetector.h"
#include <memory.h>
#include <assert.h>
#include <cstdlib>
#include <cstdio>


void CollisionPrimitive::calculateInternals()
{
	transform = body->node->_getFullTransform();
}

static inline Ogre::Real transformToAxis(
	const CollisionBox &box,
	const Ogre::Vector3 &axis
	)
{
	return
		box.halfSize.x * (axis * box.getAxis(0)).length() +
		box.halfSize.y * (axis * box.getAxis(1)).length() +
		box.halfSize.z * (axis * box.getAxis(2)).length();
}

/**
* This function checks if the two boxes overlap
* along the given axis. The final parameter toCentre
* is used to pass in the vector between the boxes centre
* points, to avoid having to recalculate it each time.
*/
static inline bool overlapOnAxis(
	const CollisionBox &one,
	const CollisionBox &two,
	const Ogre::Vector3 &axis,
	const Ogre::Vector3 &toCentre
	)
{
	// Project the half-size of one onto axis
	Ogre::Real oneProject = transformToAxis(one, axis);
	Ogre::Real twoProject = transformToAxis(two, axis);

	// Project this onto the axis
	Ogre::Real distance = (toCentre * axis).length();

	// Check for overlap
	return (distance < oneProject + twoProject);
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox intersection  method.
#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)

bool IntersectionTests::boxAndBox(
	const CollisionBox &one,
	const CollisionBox &two
	)
{
	// Find the vector between the two centres
	Ogre::Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

	return (
		// Check on box one's axes first
		TEST_OVERLAP(one.getAxis(0)) &&
		TEST_OVERLAP(one.getAxis(1)) &&
		TEST_OVERLAP(one.getAxis(2)) &&

		// And on two's
		TEST_OVERLAP(two.getAxis(0)) &&
		TEST_OVERLAP(two.getAxis(1)) &&
		TEST_OVERLAP(two.getAxis(2)) &&

		// Now on the cross products
		TEST_OVERLAP(one.getAxis(0).crossProduct(two.getAxis(0))) &&
		TEST_OVERLAP(one.getAxis(0).crossProduct(two.getAxis(1))) &&
		TEST_OVERLAP(one.getAxis(0).crossProduct(two.getAxis(2))) &&
		TEST_OVERLAP(one.getAxis(1).crossProduct(two.getAxis(0))) &&
		TEST_OVERLAP(one.getAxis(1).crossProduct(two.getAxis(1))) &&
		TEST_OVERLAP(one.getAxis(1).crossProduct(two.getAxis(2))) &&
		TEST_OVERLAP(one.getAxis(2).crossProduct(two.getAxis(0))) &&
		TEST_OVERLAP(one.getAxis(2).crossProduct(two.getAxis(1))) &&
		TEST_OVERLAP(one.getAxis(2).crossProduct(two.getAxis(2)))
		);
}
#undef TEST_OVERLAP


/*
* This function checks if the two boxes overlap
* along the given axis, returning the ammount of overlap.
* The final parameter toCentre
* is used to pass in the vector between the boxes centre
* points, to avoid having to recalculate it each time.
*/
static inline Ogre::Real penetrationOnAxis(
	const CollisionBox &one,
	const CollisionBox &two,
	const Ogre::Vector3 &axis,
	const Ogre::Vector3 &toCentre
	)
{
	// Project the half-size of one onto axis
	Ogre::Real oneProject = transformToAxis(one, axis);
	Ogre::Real twoProject = transformToAxis(two, axis);

	// Project this onto the axis
	Ogre::Real distance = (toCentre * axis).length();

	// Return the overlap (i.e. positive indicates
	// overlap, negative indicates separation).
	return oneProject + twoProject - distance;
}


static inline bool tryAxis(
	const CollisionBox &one,
	const CollisionBox &two,
	Ogre::Vector3 axis,
	const Ogre::Vector3& toCentre,
	unsigned index,

	// These values may be updated
	Ogre::Real& smallestPenetration,
	unsigned &smallestCase
	)
{
	// Make sure we have a normalized axis, and don't check almost parallel axes
	if (axis.squaredLength() < 0.0001) return true;
	axis.normalise();

	Ogre::Real penetration = penetrationOnAxis(one, two, axis, toCentre);

	if (penetration < 0) return false;
	if (penetration < smallestPenetration) {
		smallestPenetration = penetration;
		smallestCase = index;
	}
	return true;
}

void fillPointFaceBoxBox(
	const CollisionBox &one,
	const CollisionBox &two,
	const Ogre::Vector3 &toCentre,
	CollisionData *data,
	unsigned best,
	Ogre::Real pen
	)
{
	// This method is called when we know that a vertex from
	// box two is in contact with box one.

	Contact* contact = data->contacts;

	// We know which axis the collision is on (i.e. best),
	// but we need to work out which of the two faces on
	// this axis.
	Ogre::Vector3 normal = one.getAxis(best);
	if (one.getAxis(best) * toCentre >  Ogre::Vector3::ZERO)
	{
		normal = normal * -1.0f;
	}

	// Work out which vertex of box two we're colliding with.
	// Using toCentre doesn't work!
	Ogre::Vector3 vertex = two.halfSize;
	if (two.getAxis(0) * normal < Ogre::Vector3::ZERO) vertex.x = -vertex.x;
	if (two.getAxis(1) * normal < Ogre::Vector3::ZERO) vertex.y = -vertex.y;
	if (two.getAxis(2) * normal < Ogre::Vector3::ZERO) vertex.z = -vertex.z;

	// Create the contact data
	contact->contactNormal = normal;
	contact->penetration = pen;
	contact->contactPoint = two.getTransform() * vertex;
	contact->setBodyData(one.body, two.body,
		data->friction, data->restitution);
}

static inline Ogre::Vector3 contactPoint(
	const Ogre::Vector3 &pOne,
	const Ogre::Vector3 &dOne,
	Ogre::Real oneSize,
	const Ogre::Vector3 &pTwo,
	const Ogre::Vector3 &dTwo,
	Ogre::Real twoSize,

	// If this is true, and the contact point is outside
	// the edge (in the case of an edge-face contact) then
	// we use one's midpoint, otherwise we use two's.
	bool useOne)
{
	Ogre::Vector3 toSt, cOne, cTwo;
	Ogre::Real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	Ogre::Real denom, mua, mub;

	smOne = dOne.squaredLength();
	smTwo = dTwo.squaredLength();
	dpOneTwo = dTwo.dotProduct(dOne);

	toSt = pOne - pTwo;
	dpStaOne = dOne.dotProduct(toSt);
	dpStaTwo = dTwo.dotProduct(toSt);

	denom = smOne * smTwo - dpOneTwo * dpOneTwo;

	// Zero denominator indicates parrallel lines
	if (denom < 0.0001f) {
		return useOne ? pOne : pTwo;
	}

	mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
	mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

	// If either of the edges has the nearest point out
	// of bounds, then the edges aren't crossed, we have
	// an edge-face contact. Our point is on the edge, which
	// we know from the useOne parameter.
	if (mua > oneSize ||
		mua < -oneSize ||
		mub > twoSize ||
		mub < -twoSize)
	{
		return useOne ? pOne : pTwo;
	}
	else
	{
		cOne = pOne + dOne * mua;
		cTwo = pTwo + dTwo * mub;

		return cOne * 0.5 + cTwo * 0.5;
	}
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox contact generation method.
#define CHECK_OVERLAP(axis, index) \
    if (!tryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;

unsigned CollisionDetector::boxAndBox(
	const CollisionBox &one,
	const CollisionBox &two,
	CollisionData *data
	)
{
	//if (!IntersectionTests::boxAndBox(one, two)) return 0;

	// Find the vector between the two centres
	Ogre::Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

	// We start assuming there is no contact
	Ogre::Real pen = Ogre::Math::POS_INFINITY;
	unsigned best = 0xffffff;

	// Now we check each axes, returning if it gives us
	// a separating axis, and keeping track of the axis with
	// the smallest penetration otherwise.
	CHECK_OVERLAP(one.getAxis(0), 0);
	CHECK_OVERLAP(one.getAxis(1), 1);
	CHECK_OVERLAP(one.getAxis(2), 2);

	CHECK_OVERLAP(two.getAxis(0), 3);
	CHECK_OVERLAP(two.getAxis(1), 4);
	CHECK_OVERLAP(two.getAxis(2), 5);

	// Store the best axis-major, in case we run into almost
	// parallel edge collisions later
	unsigned bestSingleAxis = best;

	CHECK_OVERLAP(one.getAxis(0).crossProduct(two.getAxis(0)), 6);
	CHECK_OVERLAP(one.getAxis(0).crossProduct(two.getAxis(1)), 7);
	CHECK_OVERLAP(one.getAxis(0).crossProduct(two.getAxis(2)), 8);
	CHECK_OVERLAP(one.getAxis(1).crossProduct(two.getAxis(0)), 9);
	CHECK_OVERLAP(one.getAxis(1).crossProduct(two.getAxis(1)), 10);
	CHECK_OVERLAP(one.getAxis(1).crossProduct(two.getAxis(2)), 11);
	CHECK_OVERLAP(one.getAxis(2).crossProduct(two.getAxis(0)), 12);
	CHECK_OVERLAP(one.getAxis(2).crossProduct(two.getAxis(1)), 13);
	CHECK_OVERLAP(one.getAxis(2).crossProduct(two.getAxis(2)), 14);

	// Make sure we've got a result.
	assert(best != 0xffffff);

	// We now know there's a collision, and we know which
	// of the axes gave the smallest penetration. We now
	// can deal with it in different ways depending on
	// the case.
	if (best < 3)
	{
		// We've got a vertex of box two on a face of box one.
		fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
		data->addContacts(1);
		return 1;
	}
	else if (best < 6)
	{
		// We've got a vertex of box one on a face of box two.
		// We use the same algorithm as above, but swap around
		// one and two (and therefore also the vector between their
		// centres).
		fillPointFaceBoxBox(two, one, toCentre*-1.0f, data, best - 3, pen);
		data->addContacts(1);
		return 1;
	}
	else
	{
		// We've got an edge-edge contact. Find out which axes
		best -= 6;
		unsigned oneAxisIndex = best / 3;
		unsigned twoAxisIndex = best % 3;
		Ogre::Vector3 oneAxis = one.getAxis(oneAxisIndex);
		Ogre::Vector3 twoAxis = two.getAxis(twoAxisIndex);
		Ogre::Vector3 axis = oneAxis.crossProduct(twoAxis);
		axis.normalise();

		// The axis should point from box one to box two.
		if (axis * toCentre >  Ogre::Vector3::ZERO) axis = axis * -1.0f;

		// We have the axes, but not the edges: each axis has 4 edges parallel
		// to it, we need to find which of the 4 for each object. We do
		// that by finding the point in the centre of the edge. We know
		// its component in the direction of the box's collision axis is zero
		// (its a mid-point) and we determine which of the extremes in each
		// of the other axes is closest.
		Ogre::Vector3 ptOnOneEdge = one.halfSize;
		Ogre::Vector3 ptOnTwoEdge = two.halfSize;
		for (unsigned i = 0; i < 3; i++)
		{
			if (i == oneAxisIndex) ptOnOneEdge[i] = 0;
			else if (one.getAxis(i) * axis >  Ogre::Vector3::ZERO) ptOnOneEdge[i] = -ptOnOneEdge[i];

			if (i == twoAxisIndex) ptOnTwoEdge[i] = 0;
			else if (two.getAxis(i) * axis <  Ogre::Vector3::ZERO) ptOnTwoEdge[i] = -ptOnTwoEdge[i];
		}

		// Move them into world coordinates (they are already oriented
		// correctly, since they have been derived from the axes).
		ptOnOneEdge = one.transform * ptOnOneEdge;
		ptOnTwoEdge = two.transform * ptOnTwoEdge;

		// So we have a point and a direction for the colliding edges.
		// We need to find out point of closest approach of the two
		// line-segments.
		Ogre::Vector3 vertex = contactPoint(
			ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex],
			ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex],
			bestSingleAxis > 2
			);

		// We can fill the contact.
		Contact* contact = data->contacts;

		contact->penetration = pen;
		contact->contactNormal = axis;
		contact->contactPoint = vertex;
		contact->setBodyData(one.body, two.body,
			data->friction, data->restitution);
		data->addContacts(1);
		return 1;
	}
	return 0;
}
#undef CHECK_OVERLAP

unsigned CollisionDetector::boxAndPoint(
	const CollisionBox &box,
	const Ogre::Vector3 &point,
	CollisionData *data
	)
{
	// Transform the point into box coordinates
	Ogre::Vector3 relPt = box.transform * (point);

	Ogre::Vector3 normal;

	// Check each axis, looking for the axis on which the
	// penetration is least deep.
	Ogre::Real min_depth = box.halfSize.x - relPt.x;
	if (min_depth < 0) return 0;
	normal = box.getAxis(0) * ((relPt.x < 0) ? -1 : 1);

	Ogre::Real depth = box.halfSize.y - relPt.y;
	if (depth < 0) return 0;
	else if (depth < min_depth)
	{
		min_depth = depth;
		normal = box.getAxis(1) * ((relPt.y < 0) ? -1 : 1);
	}

	depth = box.halfSize.z - relPt.z;
	if (depth < 0) return 0;
	else if (depth < min_depth)
	{
		min_depth = depth;
		normal = box.getAxis(2) * ((relPt.z < 0) ? -1 : 1);
	}

	// Compile the contact
	Contact* contact = data->contacts;
	contact->contactNormal = normal;
	contact->contactPoint = point;
	contact->penetration = min_depth;

	// Note that we don't know what rigid body the point
	// belongs to, so we just use NULL. Where this is called
	// this value can be left, or filled in.
	contact->setBodyData(box.body, NULL,
		data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}
