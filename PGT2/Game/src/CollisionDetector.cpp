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



///This function checks if the two boxes overlap
///along the given axis, returning the ammount of overlap.
///The final parameter toCentre
///is used to pass in the vector between the boxes centre points

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

/// This method is called when we know that a vertex from
/// box two is in contact with box one.
void fillPointFaceBoxBox(
	const CollisionBox &one,
	const CollisionBox &two,
	const Ogre::Vector3 &toCentre,
	CollisionData *data,
	unsigned best,
	Ogre::Real pen
	)
{
	

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

/// This preprocessor definition is only used as a convenience
/// in the boxAndBox contact generation method.
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

#define BALL_SIZE 100
unsigned CollisionDetector::boxAndSphere(
	const CollisionBox &box,
	Ball &sphere,
	CollisionData *data
)
{
	// Transform the centre of the sphere into box coordinates
	Ogre::Vector3 centre = sphere.getPosition();
	Ogre::Vector3 relCentre = box.getTransform().inverse() * (centre);

	// Early out check to see if we can exclude the contact
	if ((relCentre.x) - BALL_SIZE > box.halfSize.x ||
		(relCentre.y) - BALL_SIZE > box.halfSize.y ||
		(relCentre.z) - BALL_SIZE > box.halfSize.z)
	{
		return 0;
	}

	Ogre::Vector3 closestPt(0, 0, 0);
	Ogre::Real dist;

	// Clamp each coordinate to the box.
	dist = relCentre.x;
	if (dist > box.halfSize.x) dist = box.halfSize.x;
	if (dist < -box.halfSize.x) dist = -box.halfSize.x;
	closestPt.x = dist;

	dist = relCentre.y;
	if (dist > box.halfSize.y) dist = box.halfSize.y;
	if (dist < -box.halfSize.y) dist = -box.halfSize.y;
	closestPt.y = dist;

	dist = relCentre.z;
	if (dist > box.halfSize.z) dist = box.halfSize.z;
	if (dist < -box.halfSize.z) dist = -box.halfSize.z;
	closestPt.z = dist;

	// Check we're in contact
	dist = (closestPt - relCentre).squaredLength();
	if (dist > BALL_SIZE * BALL_SIZE) return 0;

	// Compile the contact
	Ogre::Vector3 closestPtWorld = box.getTransform() * (closestPt);

	Contact* contact = data->contacts;
	contact->contactNormal = (closestPtWorld - centre);
	contact->contactNormal.normalise();
	contact->contactPoint = closestPtWorld;
	contact->penetration = BALL_SIZE - Ogre::Math::Sqrt(dist);
	contact->setBodyData(box.body, &sphere,
		data->friction, data->restitution);

	data->addContacts(1);
	return 1;
}
#undef BALL_SIZE


bool IntersectionTests::boxAndHalfSpace(const CollisionBox & box, Ogre::Vector3 direction, Ogre::Real offset, Face face)
{
	// Work out the projected radius of the box onto the plane direction
	Ogre::Real projectedRadius = transformToAxis(box, direction);

	// Work out how far the box is from the origin
	Ogre::Real boxDistance =
		direction.dotProduct(
		box.getAxis(3)) -
		projectedRadius;


	Ogre::Vector3* boundingBox = box.body->getBoundingBox(true);

	bool pointIsInTriangle = false;

	for (int i = 0; i < 8; i++)
	{
		if (pointAndFace(boundingBox[i], face))
		{
			pointIsInTriangle = true;
			break;
		}
	}
	// Check for the intersection

	return boxDistance <= offset && pointIsInTriangle;

}

bool IntersectionTests::pointAndFace(Ogre::Vector3 point, Face face)
{
	Ogre::Vector3 p1 = face.point1;
	Ogre::Vector3 p2 = face.point2;
	Ogre::Vector3 p3 = face.point3;


	Ogre::Real alpha = ((p2.z - p3.z)*(point.x - p3.x) + (p3.x - p2.x)*(point.z - p3.z)) /
		((p2.z - p3.z)*(p1.x - p3.x) + (p3.x - p2.x)*(p1.z - p3.z));
	Ogre::Real beta = ((p3.z - p1.z)*(point.x - p3.x) + (p1.x - p3.x)*(point.z - p3.z)) /
		((p2.z - p3.z)*(p1.x - p3.x) + (p3.x - p2.x)*(p1.z - p3.z));
	Ogre::Real gamma = 1.0f - alpha - beta;

	if (alpha > 0 && beta > 0 && gamma > 0)
		return true;
	else
		return false;
}

unsigned CollisionDetector::boxAndHalfSpace(
	const CollisionBox & box,
	CollisionData* data,
	Face face
)
{
	Ogre::Vector3 direction = face.normal;
	Ogre::Real offset = face.point1.dotProduct(direction);

	// Make sure we have contacts
	if (data->contactsLeft <= 0) return 0;

	// Check for intersection
	if (!IntersectionTests::boxAndHalfSpace(box, direction, offset, face))
	{
		return 0;
	}

	// We have an intersection, so find the intersection points. We can make
	// do with only checking vertices. If the box is resting on a plane
	// or on an edge, it will be reported as four or two contact points.

	// Go through each combination of + and - for each half-size
	static Ogre::Real mults[8][3] = { { 1,1,1 },{ -1,1,1 },{ 1,-1,1 },{ -1,-1,1 },
	{ 1,1,-1 },{ -1,1,-1 },{ 1,-1,-1 },{ -1,-1,-1 } };

	Contact* contact = data->contacts;
	unsigned contactsUsed = 0;
	for (unsigned i = 0; i < 8; i++) {

		// Calculate the position of each vertex
		Ogre::Vector3 vertexPos(mults[i][0], mults[i][1], mults[i][2]);
		vertexPos *= box.halfSize;
		vertexPos = box.transform * vertexPos;

		// Calculate the distance from the plane
		Ogre::Real vertexDistance = vertexPos.dotProduct(direction);

		// Compare this to the plane's distance
		if (vertexDistance <= offset)
		{
			// Create the contact data.

			// The contact point is halfway between the vertex and the
			// plane - we multiply the direction by half the separation
			// distance and add the vertex location.
			Ogre::Vector3 temp = direction;
			temp *= (vertexDistance - offset);
			temp += vertexPos;
		
			contact->contactPoint = temp;
			contact->contactNormal = direction;
			contact->penetration = (offset - vertexDistance);

			// Write the appropriate data
			contact->setBodyData(box.body, NULL,
				data->friction, data->restitution);

			// Move onto the next contact
			contact++;
			contactsUsed++;
			if (contactsUsed == (unsigned)data->contactsLeft) return contactsUsed;
		}
	}

	data->addContacts(contactsUsed);
	return contactsUsed;
}