#include "RigidBody.h"
#include "Helper.h"
#include "World.h"
#include <cmath>
RigidBody::RigidBody(Ogre::SceneNode* _node, Ogre::Entity* _entity)
{
	
	RigidBody::node = _node;
	RigidBody::entity = _entity;
	RigidBody::name = RigidBody::entity->getName();
	RigidBody::inverseMass = 1;
	RigidBody::dampening = 0.995;
	RigidBody::isAwake = true;
	RigidBody::canSleep = false;
	RigidBody::canCollide = true;
	RigidBody::velocity = Ogre::Vector3().ZERO;
	RigidBody::acceleration = Ogre::Vector3(0.0f, -9.81f, 0.0f);
	RigidBody::forceAccum = Ogre::Vector3().ZERO;
	RigidBody::torqueAccum = Ogre::Vector3().ZERO;
	RigidBody::rotation = Ogre::Vector3().ZERO;
	RigidBody::lastFrameAcceleration = Ogre::Vector3().ZERO;
	RigidBody::inertiaTensor = Ogre::Matrix3().ZERO;
	RigidBody::inertiaTensor.Inverse(RigidBody::inverseInertiaTensor);
	RigidBody::loadMeshInfo();
	RigidBody::createBoundingBox();

	/*
	
	if (cut)
	{
		//<----hier vult hij alle data van de mesh van de ball---->
		
	}

	*/
}

RigidBody::RigidBody(void)
{
}

void RigidBody::loadMeshInfo()
{
	Helper::getMeshInformation(entity->getMesh(), vertexCount, vertices, indexCount, indices, getPosition(), getOrientation(), node->getScale());
	int max = indexCount - 3;
	normals = new Ogre::Vector3[indexCount / 3];
	for (int i = 0; i < max; i += 3)
	{
		long index1 = indices[i];
		long index2 = indices[i + 1];
		long index3 = indices[i + 2];
		if (index1 < 50000)
		{
			Ogre::Vector3 point1 = vertices[index1];
			Ogre::Vector3 point2 = vertices[index2];
			Ogre::Vector3 point3 = vertices[index3];
			Ogre::Vector3 middlePoint = (point1 + point2 + point3) / 3;
			Face face = Face();
			face.point1 = point1;
			face.point2 = point2;
			face.point3 = point3;
			face.normal = Helper::normalVector(point1, point2, point3);
			faces.push_back(face);
			normals[faceCount] = face.normal;

			faceCount++;
		}

	}

}

void RigidBody::setPosition(Ogre::Vector3 position)
{
	RigidBody::node->setPosition(position);
}

void RigidBody::setVelocity(Ogre::Vector3 velocity)
{
	RigidBody::velocity = velocity;
}

Ogre::Vector3 RigidBody::getVelocity()
{
	return RigidBody::velocity;
}

Ogre::Vector3 RigidBody::getRotation()
{
	return RigidBody::rotation;
}

Ogre::Vector3 RigidBody::getPosition()
{
	return RigidBody::node->getPosition();
}

Ogre::Vector3 RigidBody::getLastFrameAcceleration()
{
	return RigidBody::lastFrameAcceleration;
}

Ogre::Matrix3 RigidBody::getInverseInertiaTensorWorld()
{
	return RigidBody::inverseInertiaTensorWorld;
}

void RigidBody::addVelocity(Ogre::Vector3 vel)
{
	RigidBody::velocity += vel;
}

void RigidBody::addRotation(Ogre::Vector3 rot)
{
	RigidBody::rotation += rot;
}

void RigidBody::setOrientation(Ogre::Quaternion& orientation)
{
	orientation.normalise();
	RigidBody::node->setOrientation(orientation);
}

Ogre::Quaternion RigidBody::getOrientation()
{
	return RigidBody::node->getOrientation();
}

void RigidBody::addForce(Ogre::Vector3 force)
{
	RigidBody::forceAccum += force;
}

void RigidBody::addTorque(Ogre::Vector3 torque)
{
	RigidBody::torqueAccum = torque;
}

void RigidBody::addForceAtBodyPoint(Ogre::Vector3 force, Ogre::Vector3 point)
{
	// Convert to coordinates relative to center of mass.
	Ogre::Vector3 pt = RigidBody::node->convertLocalToWorldPosition(point);
	RigidBody::addForceAtPoint(force, pt);
}

void RigidBody::addForceAtPoint(Ogre::Vector3 force, Ogre::Vector3 point)
{
	// Convert to coordinates relative to center of mass.
	Ogre::Vector3 pt = point;
	Ogre::Vector3 pos = RigidBody::getPosition();
	pt -= pos;

	RigidBody::forceAccum += force;
	RigidBody::torqueAccum += Ogre::Vector3(pt.y*force.z - pt.z*force.y,
											pt.z*force.x - pt.x*force.z,
											pt.x*force.y - pt.y*force.x);  ;
}

void RigidBody::setIsAwake(const bool awake)
{
	RigidBody::isAwake = awake;
}


void RigidBody::_transformInertiaTensor(Ogre::Matrix3 &iitWorld,
	 Ogre::Quaternion q,
	 Ogre::Matrix3 &iitBody,
	 Ogre::Matrix4 &rotmat)
{
	Ogre::Real t4 = rotmat[0][0] * iitBody[0][0] +
		rotmat[0][1] * iitBody[1][0] +
		rotmat[0][2] * iitBody[2][0];
	Ogre::Real t9 = rotmat[0][0] * iitBody[0][1] +
		rotmat[0][1] * iitBody[1][1] +
		rotmat[0][2] * iitBody[2][1];
	Ogre::Real t14 = rotmat[0][0] * iitBody[0][2] +
		rotmat[0][1] * iitBody[1][2] +
		rotmat[0][2] * iitBody[2][2];
	Ogre::Real t28 = rotmat[1][0] * iitBody[0][0] +
		rotmat[1][1] * iitBody[1][0] +
		rotmat[1][2] * iitBody[2][0];
	Ogre::Real t33 = rotmat[1][0] * iitBody[0][1] +
		rotmat[1][1] * iitBody[1][1] +
		rotmat[1][2] * iitBody[2][1];
	Ogre::Real t38 = rotmat[1][0] * iitBody[0][2] +
		rotmat[1][1] * iitBody[1][2] +
		rotmat[1][2] * iitBody[2][2];
	Ogre::Real t52 = rotmat[2][0] * iitBody[0][0] +
		rotmat[2][1] * iitBody[1][0] +
		rotmat[2][2] * iitBody[2][0];
	Ogre::Real t57 = rotmat[2][0] * iitBody[0][1] +
		rotmat[2][1] * iitBody[1][1] +
		rotmat[2][2] * iitBody[2][1];
	Ogre::Real t62 = rotmat[2][0] * iitBody[0][2] +
		rotmat[2][1] * iitBody[1][2] +
		rotmat[2][2] * iitBody[2][2];

	iitWorld[0][0] = t4*rotmat[0][0] +
		t9*rotmat[0][1] +
		t14*rotmat[0][2];
	iitWorld[0][1] = t4*rotmat[1][0] +
		t9*rotmat[1][1] +
		t14*rotmat[1][2];
	iitWorld[0][2] = t4*rotmat[2][0] +
		t9*rotmat[2][1] +
		t14*rotmat[2][2];
	iitWorld[1][0] = t28*rotmat[0][0] +
		t33*rotmat[0][1] +
		t38*rotmat[0][2];
	iitWorld[1][1] = t28*rotmat[1][0] +
		t33*rotmat[1][1] +
		t38*rotmat[1][2];
	iitWorld[1][2] = t28*rotmat[2][0] +
		t33*rotmat[2][1] +
		t38*rotmat[2][2];
	iitWorld[2][0] = t52*rotmat[0][0] +
		t57*rotmat[0][1] +
		t62*rotmat[0][2];
	iitWorld[2][1] = t52*rotmat[1][0] +
		t57*rotmat[1][1] +
		t62*rotmat[1][2];
	iitWorld[2][2] = t52*rotmat[2][0] +
		t57*rotmat[2][1] +
		t62*rotmat[2][2];
}

void RigidBody::clearAccumulators()
{
	RigidBody::forceAccum = Ogre::Vector3::ZERO;
	RigidBody::torqueAccum = Ogre::Vector3::ZERO;
}

void RigidBody::calculateDerivedData()
{
	// Calculate the transform matrix for the body.
	RigidBody::transformMatrix = RigidBody::node->_getFullTransform();

	// Calculate the inertiaTensor in world space.
	_transformInertiaTensor(RigidBody::inverseInertiaTensorWorld,
		RigidBody::getOrientation(),
		RigidBody::inverseInertiaTensor,
		RigidBody::transformMatrix);

}

bool RigidBody::hitBoxContainsPoint(Ogre::Vector3 point)
{
	Ogre::Vector3* boundingBox = RigidBody::getBoundingBox(false);

	Ogre::Matrix3 rotMatrix;
	getOrientation().ToRotationMatrix(rotMatrix);
	rotMatrix = rotMatrix.Inverse();
	Ogre::Vector3 translatedPoint = point * rotMatrix;
	Ogre::Vector3 point1 = boundingBox[0];
	Ogre::Vector3 point2 = boundingBox[1];
	if (point1.x <= translatedPoint.x && translatedPoint.x <= point2.x && point1.y <= translatedPoint.y && translatedPoint.y <= point2.y) {
		return true;
	}
	return false;
}


void RigidBody::integrate(Ogre::Real delta)
{
	if (RigidBody::isAwake)
	{
		//calculate acceleration with mass and force
		//TODO calculate angular acceleration with Tensor and Torque
		RigidBody::lastFrameAcceleration = RigidBody::acceleration;
		RigidBody::lastFrameAcceleration += (RigidBody::forceAccum * RigidBody::inverseMass);

		Ogre::Vector3 AngularAcceleration = (RigidBody::inverseInertiaTensorWorld * RigidBody::torqueAccum);

		//Update velocity with time and acceleration
		RigidBody::velocity += (RigidBody::lastFrameAcceleration);
		RigidBody::rotation += (AngularAcceleration);

		//dampen the movement so it stops eventually !Temporary fix pls watch me
		RigidBody::velocity *= RigidBody::dampening;
		RigidBody::rotation *= RigidBody::dampening;

		//Move Rigidbody with velocity and time
		Ogre::Vector3 tempPos = RigidBody::getPosition();
		tempPos += (RigidBody::velocity * delta);
		RigidBody::setPosition(tempPos);

		Ogre::Quaternion q(0,
			RigidBody::rotation.x * delta,
			RigidBody::rotation.y * delta,
			RigidBody::rotation.z * delta);
		Ogre::Quaternion tempOrien = RigidBody::getOrientation();
		q = q * tempOrien;
		tempOrien.w += q.w * ((Ogre::Real)0.5);
		tempOrien.x += q.x * ((Ogre::Real)0.5);
		tempOrien.y += q.y * ((Ogre::Real)0.5);
		tempOrien.z += q.z * ((Ogre::Real)0.5);

		RigidBody::setOrientation(tempOrien);

		calculateDerivedData();

		clearAccumulators();

		//TODO Calculate total movement and check if under benchmark: IsAwake = false
	}
}

/// Returns true if this object is moveable.
bool RigidBody::hasFiniteMass()
{
	if (RigidBody::inverseMass == 0)
	{
		return false;
	}
	return true;
}

/// Get the mass.
Ogre::Real RigidBody::getMass()
{
	return 1 / RigidBody::inverseMass;
}

/// Get the inverse Mass.
Ogre::Real RigidBody::getInverseMass()
{
	return RigidBody::inverseMass;
}

void RigidBody::createBoundingBox()
{
	Ogre::Vector3 maxSize = Ogre::Vector3(10000000, 10000000, 10000000);
	Ogre::Vector3 minSize = Ogre::Vector3(-10000000, -10000000, -10000000);

	for (int i = 0; i < vertexCount; i++)
	{
		if (vertices[i].x < maxSize.x)
			maxSize.x = vertices[i].x;
		if (vertices[i].x > minSize.x)
			minSize.x = vertices[i].x;

		if (vertices[i].y < maxSize.y)
			maxSize.y = vertices[i].y;
		if (vertices[i].y > minSize.y)
			minSize.y = vertices[i].y;

		if (vertices[i].z < maxSize.z)
			maxSize.z = vertices[i].z;
		if (vertices[i].z > minSize.z)
			minSize.z = vertices[i].z;
	}

	boundingBox.push_back(maxSize);
	boundingBox.push_back(minSize);
	boundingBox.push_back(Ogre::Vector3(maxSize.x, maxSize.y, minSize.y));
	boundingBox.push_back(Ogre::Vector3(maxSize.x, minSize.y, maxSize.y));
	boundingBox.push_back(Ogre::Vector3(minSize.x, maxSize.y, maxSize.y));
	boundingBox.push_back(Ogre::Vector3(minSize.x, minSize.y, maxSize.y));
	boundingBox.push_back(Ogre::Vector3(minSize.x, maxSize.y, minSize.y));
	boundingBox.push_back(Ogre::Vector3(maxSize.x, minSize.y, minSize.y));
}


/**
gets boundingbox coordinates in either world space or local(???) space
*/
Ogre::Vector3* RigidBody::getBoundingBox(bool worldPosition)
{
	Ogre::Vector3 retBoundingBox[8];
	Ogre::Matrix3 rotMatrix;
	getOrientation().ToRotationMatrix(rotMatrix);
	for (int i = 0; i < boundingBox.size(); i++)
	{
		retBoundingBox[i] = boundingBox[i];
		if (worldPosition)
		{
			retBoundingBox[i] += RigidBody::getPosition() * rotMatrix;
		}
	}
	Ogre::Vector3* a = retBoundingBox;
	return a;
}

bool RigidBody::setAndCheckIsAwake()
{
	if (RigidBody::velocity.length() + RigidBody::rotation.length() < 8)
	{
		RigidBody::setIsAwake(false);
		return false;
	}
	else
	{
		RigidBody::setIsAwake(true);
		return true;
	}
}

void RigidBody::setInertiaTensor(const Ogre::Matrix3& inertiaTensor)
{
	RigidBody::inverseInertiaTensor = inertiaTensor.Inverse();
}

/// \brief Calculate resulting vector after subtraction.
///
/// TODO: this method could be placed in a math helper class.
void sub3(const Ogre::Vector3* first, const Ogre::Vector3* second, Ogre::Vector3* result)
{
	result->x = first->x - second->x;
	result->y = first->y - second->y;
	result->z = first->z - second->z;
}

/// \brief Calculate the dot product of two vectors.
///
/// TODO: This method could be placed in a math helper class.
float dot3(const Ogre::Vector3* first, const Ogre::Vector3* second)
{
	float dot = first->x * second->x + first->y * second->y + first->z * second->z;

	return dot;
}

void lerp3(const Ogre::Vector3* from, const Ogre::Vector3* to, float t, Ogre::Vector3* result)
{
	result->x = from->x + t * (to->x - from->x);
	result->y = from->y + t * (to->y - from->y);
	result->z = from->z + t * (to->z - from->z);
}

float linePlaneCoefficient(const Ogre::Vector3* linePoint, const Ogre::Vector3* lineDir, const Ogre::Vector3* planeNormal, const Ogre::Vector3* planePoint)
{
	Ogre::Vector3 diff;
	float dot1;
	float dot2;
	float result;

	sub3(planePoint, linePoint, &diff);
	dot1 = dot3(planeNormal, &diff);
	dot2 = dot3(planeNormal, lineDir);
	result = dot1 / dot2;

	return result;
}

/// \brief Slice a mesh into two new meshes.
///
/// This function will be called when the wrecking ball hits a building or other destroyable mesh.
/// What should happen is the mesh being cut according to the force of the ball. Right now, a random plane
/// is used to indicute the cutting edge, but the of cutting the mesh with a slicing plane still applies.
///
/// Using the slicing plane, each intersection vertex can be found. Using those, the mesh is seperated at
/// these points, and new indices + faces are created for the missing parts of the two new meshes.
void RigidBody::cut(Ogre::Vector3 planePoint, Ogre::Vector3 planeNormal)
{
	
	int faceCount = indexCount / 3;

	// The maximum number of vertices for the new meshes is vertexCount + faceCount * 2
	// (two points of intersection for each face)
	int newVertexCount = vertexCount;
	int newVertexMax = vertexCount + faceCount * 2;

	Ogre::Vector3* newVertices = new Ogre::Vector3[newVertexMax];
	Ogre::Vector3* newNormals = new Ogre::Vector3[newVertexMax];

	memcpy(newVertices, vertices, vertexCount * sizeof(Ogre::Vector3));
	memcpy(newNormals, normals, vertexCount * sizeof(Ogre::Vector3));

	// The maximum number of indices for the new meshes is indexCount + faceCount * 9
	// (each face could be split up into three faces)
	int newIndexCount = indexCount;
	int newIndexMax = indexCount + faceCount * 9;

	int leftIndexCount = 0;
	int* leftIndices = new int[newIndexMax];

	int rightIndexCount = 0;
	int* rightIndices = new int[newIndexMax];
	
	// Array to store the array index of the new intersection vertices, so we can loop through them.
	int intersectionMax = faceCount * 2;
	int intersectionCount = 0;
	int* intersectionsArray = new int[intersectionMax];
	
	// Iterate through each face and decide which list to put it in and whether to divide it
	for (int i = 0; i < faceCount; ++i)
	{
		int i1 = indices[i * 3 + 0];
		int i2 = indices[i * 3 + 1];
		int i3 = indices[i * 3 + 2];

		Ogre::Vector3* v1 = &vertices[i1];
		Ogre::Vector3* v2 = &vertices[i2];
		Ogre::Vector3* v3 = &vertices[i3];

		// Check if each vertex is to the left of the plane
		Ogre::Vector3 v1ToPlane;
		Ogre::Vector3 v2ToPlane;
		Ogre::Vector3 v3ToPlane;

		sub3(&planePoint, v1, &v1ToPlane);
		sub3(&planePoint, v2, &v2ToPlane);
		sub3(&planePoint, v3, &v3ToPlane);

		bool v1Left = dot3(&v1ToPlane, &planeNormal) < 0;
		bool v2Left = dot3(&v2ToPlane, &planeNormal) < 0;
		bool v3Left = dot3(&v3ToPlane, &planeNormal) < 0;

		int pointsToLeft = (v1Left ? 1 : 0) + (v2Left ? 1 : 0) + (v3Left ? 1 : 0);

		// All terrainVertices to left
		if (pointsToLeft == 3)
		{
			// Add all terrainVertices to left
			leftIndices[leftIndexCount++] = i1;
			leftIndices[leftIndexCount++] = i2;
			leftIndices[leftIndexCount++] = i3;
		}
		// All terrainVertices to right
		else if (pointsToLeft == 0)
		{
			// Add all terrainVertices to right
			rightIndices[rightIndexCount++] = i1;
			rightIndices[rightIndexCount++] = i2;
			rightIndices[rightIndexCount++] = i3;
		}
		// One vertex to left
		else if (pointsToLeft == 1)
		{
			if (v1Left)
			{
				Ogre::Vector3 line1, line2;
				Ogre::Vector3 intersect1, intersect2;

				int intersect1index, intersect2index;
				float intersect1coeff, intersect2coeff;

				int ia = i1;
				int ib = i2;
				int ic = i3;

				const Ogre::Vector3* a = &newVertices[ia];
				const Ogre::Vector3* b = &newVertices[ib];
				const Ogre::Vector3* c = &newVertices[ic];

				sub3(b, a, &line1);
				sub3(c, a, &line2);

				// Calculate lerp coefficient for intersections
				intersect1coeff = linePlaneCoefficient(a, &line1, &planeNormal, &planePoint);
				intersect2coeff = linePlaneCoefficient(a, &line2, &planeNormal, &planePoint);

				// Calculate intersections
				lerp3(a, b, intersect1coeff, &intersect1);
				lerp3(a, c, intersect2coeff, &intersect2);

				intersect1index = newVertexCount++;
				intersect2index = newVertexCount++;

				// Store intersection index in the intersection array
				intersectionsArray[intersectionCount++] = intersect1index;
				intersectionsArray[intersectionCount++] = intersect2index;

				// Save intersections as new terrainVertices
				newVertices[intersect1index] = intersect1;
				newVertices[intersect2index] = intersect2;

				lerp3(&normals[ia], &normals[ib], intersect1coeff, &newNormals[intersect1index]);
				lerp3(&normals[ia], &normals[ic], intersect2coeff, &newNormals[intersect2index]);

				// Add triangles
				// Left
				leftIndices[leftIndexCount++] = i1;
				leftIndices[leftIndexCount++] = intersect1index;
				leftIndices[leftIndexCount++] = intersect2index;

				// Right
				rightIndices[rightIndexCount++] = intersect1index;
				rightIndices[rightIndexCount++] = i2;
				rightIndices[rightIndexCount++] = i3;

				rightIndices[rightIndexCount++] = intersect1index;
				rightIndices[rightIndexCount++] = i3;
				rightIndices[rightIndexCount++] = intersect2index;
			}
			else if (v2Left)
			{
				Ogre::Vector3 line1, line2;
				Ogre::Vector3 intersect1, intersect2;
				int intersect1index, intersect2index;
				float intersect1coeff, intersect2coeff;

				int ia = i2;
				int ib = i1;
				int ic = i3;

				const Ogre::Vector3* a = &newVertices[ia];
				const Ogre::Vector3* b = &newVertices[ib];
				const Ogre::Vector3* c = &newVertices[ic];

				sub3(b, a, &line1);
				sub3(c, a, &line2);

				// Calculate lerp coefficient for intersections
				intersect1coeff = linePlaneCoefficient(a, &line1, &planeNormal, &planePoint);
				intersect2coeff = linePlaneCoefficient(a, &line2, &planeNormal, &planePoint);

				// Calculate intersections
				lerp3(a, b, intersect1coeff, &intersect1);
				lerp3(a, c, intersect2coeff, &intersect2);

				intersect1index = newVertexCount++;
				intersect2index = newVertexCount++;

				// Store intersection index in the intersection array
				intersectionsArray[intersectionCount++] = intersect1index;
				intersectionsArray[intersectionCount++] = intersect2index;

				// Save intersections as new terrainVertices
				newVertices[intersect1index] = intersect1;
				newVertices[intersect2index] = intersect2;

				lerp3(&normals[ia], &normals[ib], intersect1coeff, &newNormals[intersect1index]);
				lerp3(&normals[ia], &normals[ic], intersect2coeff, &newNormals[intersect2index]);

				// Add triangles
				// Left
				leftIndices[leftIndexCount++] = i2;
				leftIndices[leftIndexCount++] = intersect2index;
				leftIndices[leftIndexCount++] = intersect1index;

				// Right
				rightIndices[rightIndexCount++] = intersect1index;
				rightIndices[rightIndexCount++] = intersect2index;
				rightIndices[rightIndexCount++] = i3;

				rightIndices[rightIndexCount++] = intersect1index;
				rightIndices[rightIndexCount++] = i3;
				rightIndices[rightIndexCount++] = i1;
			}
			else
			{
				Ogre::Vector3 line1, line2;
				Ogre::Vector3 intersect1, intersect2;
				int intersect1index, intersect2index;
				float intersect1coeff, intersect2coeff;

				int ia = i3;
				int ib = i1;
				int ic = i2;

				const Ogre::Vector3* a = &newVertices[ia];
				const Ogre::Vector3* b = &newVertices[ib];
				const Ogre::Vector3* c = &newVertices[ic];

				sub3(b, a, &line1);
				sub3(c, a, &line2);

				// Calculate lerp coefficient for intersections
				intersect1coeff = linePlaneCoefficient(a, &line1, &planeNormal, &planePoint);
				intersect2coeff = linePlaneCoefficient(a, &line2, &planeNormal, &planePoint);

				// Calculate intersections
				lerp3(a, b, intersect1coeff, &intersect1);
				lerp3(a, c, intersect2coeff, &intersect2);

				intersect1index = newVertexCount++;
				intersect2index = newVertexCount++;

				// Store intersection index in the intersection array
				intersectionsArray[intersectionCount++] = intersect1index;
				intersectionsArray[intersectionCount++] = intersect2index;

				// Save intersections as new terrainVertices
				newVertices[intersect1index] = intersect1;
				newVertices[intersect2index] = intersect2;

				lerp3(&normals[ia], &normals[ib], intersect1coeff, &newNormals[intersect1index]);
				lerp3(&normals[ia], &normals[ic], intersect2coeff, &newNormals[intersect2index]);

				// Add triangles
				// Left
				leftIndices[leftIndexCount++] = intersect1index;
				leftIndices[leftIndexCount++] = intersect2index;
				leftIndices[leftIndexCount++] = i3;

				// Right
				rightIndices[rightIndexCount++] = i2;
				rightIndices[rightIndexCount++] = intersect2index;
				rightIndices[rightIndexCount++] = intersect1index;

				rightIndices[rightIndexCount++] = i2;
				rightIndices[rightIndexCount++] = intersect1index;
				rightIndices[rightIndexCount++] = i1;
			}
		}
		// Two terrainVertices to left
		else if (pointsToLeft == 2)
		{
			if (!v1Left)
			{
				Ogre::Vector3 line1, line2;
				Ogre::Vector3 intersect1, intersect2;
				int intersect1index, intersect2index;
				float intersect1coeff, intersect2coeff;

				int ia = i1;
				int ib = i2;
				int ic = i3;

				const Ogre::Vector3* a = &newVertices[ia];
				const Ogre::Vector3* b = &newVertices[ib];
				const Ogre::Vector3* c = &newVertices[ic];

				sub3(b, a, &line1);
				sub3(c, a, &line2);

				// Calculate lerp coefficient for intersections
				intersect1coeff = linePlaneCoefficient(a, &line1, &planeNormal, &planePoint);
				intersect2coeff = linePlaneCoefficient(a, &line2, &planeNormal, &planePoint);

				// Calculate intersections
				lerp3(a, b, intersect1coeff, &intersect1);
				lerp3(a, c, intersect2coeff, &intersect2);

				intersect1index = newVertexCount++;
				intersect2index = newVertexCount++;

				// Store intersection index in the intersection array
				intersectionsArray[intersectionCount++] = intersect1index;
				intersectionsArray[intersectionCount++] = intersect2index;

				// Save intersections as new terrainVertices
				newVertices[intersect1index] = intersect1;
				newVertices[intersect2index] = intersect2;

				lerp3(&normals[ia], &normals[ib], intersect1coeff, &newNormals[intersect1index]);
				lerp3(&normals[ia], &normals[ic], intersect2coeff, &newNormals[intersect2index]);

				// Add triangles
				// Right
				rightIndices[rightIndexCount++] = i1;
				rightIndices[rightIndexCount++] = intersect1index;
				rightIndices[rightIndexCount++] = intersect2index;

				// Left
				leftIndices[leftIndexCount++] = intersect1index;
				leftIndices[leftIndexCount++] = i2;
				leftIndices[leftIndexCount++] = i3;

				leftIndices[leftIndexCount++] = intersect1index;
				leftIndices[leftIndexCount++] = i3;
				leftIndices[leftIndexCount++] = intersect2index;
			}
			else if (!v2Left)
			{
				Ogre::Vector3 line1, line2;
				Ogre::Vector3 intersect1, intersect2;
				int intersect1index, intersect2index;
				float intersect1coeff, intersect2coeff;

				int ia = i2;
				int ib = i1;
				int ic = i3;

				const Ogre::Vector3* a = &newVertices[ia];
				const Ogre::Vector3* b = &newVertices[ib];
				const Ogre::Vector3* c = &newVertices[ic];

				sub3(b, a, &line1);
				sub3(c, a, &line2);

				// Calculate lerp coefficient for intersections
				intersect1coeff = linePlaneCoefficient(a, &line1, &planeNormal, &planePoint);
				intersect2coeff = linePlaneCoefficient(a, &line2, &planeNormal, &planePoint);

				// Calculate intersections
				lerp3(a, b, intersect1coeff, &intersect1);
				lerp3(a, c, intersect2coeff, &intersect2);

				intersect1index = newVertexCount++;
				intersect2index = newVertexCount++;

				// Store intersection index in the intersection array
				intersectionsArray[intersectionCount++] = intersect1index;
				intersectionsArray[intersectionCount++] = intersect2index;

				// Save intersections as new terrainVertices
				newVertices[intersect1index] = intersect1;
				newVertices[intersect2index] = intersect2;

				lerp3(&normals[ia], &normals[ib], intersect1coeff, &newNormals[intersect1index]);
				lerp3(&normals[ia], &normals[ic], intersect2coeff, &newNormals[intersect2index]);

				// Add triangles
				// Right
				rightIndices[rightIndexCount++] = i2;
				rightIndices[rightIndexCount++] = intersect2index;
				rightIndices[rightIndexCount++] = intersect1index;

				// Left
				leftIndices[leftIndexCount++] = intersect1index;
				leftIndices[leftIndexCount++] = intersect2index;
				leftIndices[leftIndexCount++] = i3;

				leftIndices[leftIndexCount++] = intersect1index;
				leftIndices[leftIndexCount++] = i3;
				leftIndices[leftIndexCount++] = i1;
			}
			else if (!v3Left)
			{
				Ogre::Vector3 line1, line2;
				Ogre::Vector3 intersect1, intersect2;
				int intersect1index, intersect2index;
				float intersect1coeff, intersect2coeff;

				int ia = i3;
				int ib = i1;
				int ic = i2;

				const Ogre::Vector3* a = &newVertices[ia];
				const Ogre::Vector3* b = &newVertices[ib];
				const Ogre::Vector3* c = &newVertices[ic];

				sub3(b, a, &line1);
				sub3(c, a, &line2);

				// Calculate lerp coefficient for intersections
				intersect1coeff = linePlaneCoefficient(a, &line1, &planeNormal, &planePoint);
				intersect2coeff = linePlaneCoefficient(a, &line2, &planeNormal, &planePoint);

				// Calculate intersections
				lerp3(a, b, intersect1coeff, &intersect1);
				lerp3(a, c, intersect2coeff, &intersect2);

				intersect1index = newVertexCount++;
				intersect2index = newVertexCount++;

				// Store intersection index in the intersection array
				intersectionsArray[intersectionCount++] = intersect1index;
				intersectionsArray[intersectionCount++] = intersect2index;

				// Save intersections as new terrainVertices
				newVertices[intersect1index] = intersect1;
				newVertices[intersect2index] = intersect2;

				lerp3(&normals[ia], &normals[ib], intersect1coeff, &newNormals[intersect1index]);
				lerp3(&normals[ia], &normals[ic], intersect2coeff, &newNormals[intersect2index]);

				// Add triangles
				// Right
				rightIndices[rightIndexCount++] = intersect1index;
				rightIndices[rightIndexCount++] = intersect2index;
				rightIndices[rightIndexCount++] = i3;

				// Left
				leftIndices[leftIndexCount++] = i2;
				leftIndices[leftIndexCount++] = intersect2index;
				leftIndices[leftIndexCount++] = intersect1index;

				leftIndices[leftIndexCount++] = i2;
				leftIndices[leftIndexCount++] = intersect1index;
				leftIndices[leftIndexCount++] = i1;
			}
		}
		else
		{
			int j = 0;
		}
	}

	// updating newVertices, leftIndices and rightIndices arrays with new array sizes because of soon to be added data for the missing faces.

	// Adding new middle point for filling in the missing faces
	newVertexMax++;
	Ogre::Vector3* newArr = new Ogre::Vector3[newVertexMax];
	std::copy(newVertices, newVertices + std::min(newVertexMax - 1, newVertexMax), newArr);
	delete[] newVertices;
	newVertices = newArr;
	int middlePointIndex = newVertexCount++;
	newVertices[middlePointIndex] = planePoint;

	// When creating missing intersection faces: Amount of new indices added = intersectionPoints * 3
	// intersectionCount--;
	int missingFacesIndices = intersectionCount * 3;

	// copy left and right indice index arrays, size needs to be increased to account for the new missing face indices
	int* newArr2 = new int[newIndexMax + missingFacesIndices];
	std::copy(leftIndices, leftIndices + std::min(newIndexMax, newIndexMax + missingFacesIndices), newArr2);
	delete[] leftIndices;
	leftIndices = newArr2;

	int* newArr3 = new int[newIndexMax + missingFacesIndices];
	std::copy(rightIndices, rightIndices + std::min(newIndexMax, newIndexMax + missingFacesIndices), newArr3);
	delete[] rightIndices;
	rightIndices = newArr3;

	// Todo: Refactor into function so we can reuse it for left and right
	Ogre::Vector3* leftVertices2 = new Ogre::Vector3[newVertexCount];
	Ogre::Vector3* leftNormals2 = new Ogre::Vector3[newVertexCount];
	int* leftIndices2 = new int[newIndexMax + missingFacesIndices];
	int leftVertexCount2 = newVertexCount;
	int leftIndexCount2 = leftIndexCount + missingFacesIndices;

	fillIntersectionFaces(leftIndices, leftIndexCount, intersectionsArray, intersectionCount, middlePointIndex);

	memcpy(leftVertices2, newVertices, leftVertexCount2 * sizeof(Ogre::Vector3));
	memcpy(leftNormals2, newNormals, leftVertexCount2 * sizeof(Ogre::Vector3));
	memcpy(leftIndices2, leftIndices, leftIndexCount2 * sizeof(int));

	Ogre::Entity* leftEntity = World::createCustomEntity(leftVertices2, leftIndices2, leftVertexCount2, leftIndexCount2, entity->getSubEntity(0)->getMaterialName());
	//leftBody->canCollide = false;
	//leftBody->addForce(Ogre::Vector3(5, 0, 0) * 10);
	//leftBody->addTorque(Ogre::Vector3(1, 0, 0.4f));

	Ogre::Vector3* rightVertices2 = new Ogre::Vector3[newVertexCount];
	Ogre::Vector3* rightNormals2 = new Ogre::Vector3[newVertexCount];
	int* rightIndices2 = new int[newIndexMax + missingFacesIndices];
	int rightVertexCount2 = newVertexCount;
	int rightIndexCount2 = rightIndexCount + missingFacesIndices;

	fillIntersectionFaces(rightIndices, rightIndexCount, intersectionsArray, intersectionCount, middlePointIndex);

	memcpy(rightVertices2, newVertices, rightVertexCount2 * sizeof(Ogre::Vector3));
	memcpy(rightNormals2, newNormals, rightVertexCount2 * sizeof(Ogre::Vector3));
	memcpy(rightIndices2, rightIndices, rightIndexCount * sizeof(int));

	Ogre::Entity* rightEntity = World::createCustomEntity(rightVertices2, rightIndices2, rightVertexCount2, rightIndexCount2, entity->getSubEntity(0)->getMaterialName());
	

	Ogre::SceneNode *leftNode = World::mSceneMgr->getRootSceneNode()->createChildSceneNode();
	leftNode->setPosition(Ogre::Vector3(0, 0, 0));
	leftNode->attachObject(leftEntity);
	Ogre::SceneNode *rightNode = World::mSceneMgr->getRootSceneNode()->createChildSceneNode();
	rightNode->setPosition(Ogre::Vector3(0, 0, 0));
	rightNode->attachObject(rightEntity);


	RigidBody *leftBody = new RigidBody(leftNode, leftEntity);
	leftBody->canCollide = false;
	leftBody->addForce(Ogre::Vector3(-5, 0, 0) * 10);
	leftBody->addRotation(Ogre::Vector3(1, -0.4f, 0));

	CollisionBox *leftBox = new CollisionBox();
	leftBox->body = leftBody;
	leftBox->halfSize = leftNode->getScale();
	leftBox->body->calculateDerivedData();
	leftBox->calculateInternals();

	World::addCollisionBox(leftBox);

	RigidBody *rightBody = new RigidBody(rightNode, rightEntity);
	rightBody->canCollide = false;
	rightBody->addForce(Ogre::Vector3(-5, 0, 0) * 10);
	rightBody->addRotation(Ogre::Vector3(-1, 0.4f, 0));

	CollisionBox *rightBox = new CollisionBox();
	rightBox->body = rightBody;
	rightBox->halfSize = rightNode->getScale();
	rightBox->body->calculateDerivedData();
	rightBox->calculateInternals();

	World::addCollisionBox(rightBox);

	delete newVertices;
	delete newNormals;
	delete leftIndices;
	delete rightIndices;


	delete leftVertices2;
	delete leftNormals2;
	delete leftIndices2;

	delete rightVertices2;
	delete rightNormals2;
	delete rightIndices2;
}

/// \brief Fill in the missing faces on the intersection plane after slicing a mesh.
///
/// When slicing a mesh with the cut() method, the place where the meshes are cut is not yet filled with faces.
/// This results in a visual bug and collision issues.
/// This method will create new faces on the place where the mesh is cut, using the vertices that where created
/// in the cut method along the slicing plane edge. To create triangular faces from these intersection vertices,
/// indices are created between them and planePoint, because it is always exactly in the middle of all the other vertices.
void RigidBody::fillIntersectionFaces(int* &_indicesArr, int &_indexCount, int* intersectionsArray, int intersectionCount, int middlePointIndex)
{
	for (int i = 0; i < intersectionCount; i += 2)
	{

		_indicesArr[_indexCount++] = intersectionsArray[i];
		_indicesArr[_indexCount++] = middlePointIndex;
		_indicesArr[_indexCount++] = intersectionsArray[i + 1];

		_indicesArr[_indexCount++] = middlePointIndex;
		_indicesArr[_indexCount++] = intersectionsArray[i];
		_indicesArr[_indexCount++] = intersectionsArray[i + 1];
	}
}





