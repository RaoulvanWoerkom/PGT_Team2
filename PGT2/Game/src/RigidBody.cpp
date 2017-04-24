#include "RigidBody.h"
#include "Helper.h"
#include <cmath>
RigidBody::RigidBody(Ogre::SceneNode* _node, Ogre::Entity* _entity, Ogre::SceneManager* _manager, bool cut)
{
	RigidBody::node = _node;
	RigidBody::entity = _entity;
	RigidBody::inverseMass = 1;
	RigidBody::dampening = 0.995;
	RigidBody::isAwake = true;
	RigidBody::canSleep = false;
	RigidBody::velocity = Ogre::Vector3().ZERO;
	RigidBody::acceleration = Ogre::Vector3().ZERO;
	RigidBody::forceAccum = Ogre::Vector3().ZERO;
	RigidBody::torqueAccum = Ogre::Vector3().ZERO;
	RigidBody::rotation = Ogre::Vector3().ZERO;
	RigidBody::lastFrameAcceleration = Ogre::Vector3().ZERO;
	RigidBody::inertiaTensor = Ogre::Matrix3(0.4f, 0, 0,
											 0, 0.4f, 0,
											 0 , 0, 0.4f );
	RigidBody::inertiaTensor.Inverse(RigidBody::inverseInertiaTensor);
	mSceneMgr = _manager;
	


	
	
	//memcpy(newNormals, normals, vertexCount * sizeof(Ogre::Vector3));
	//delete newNormals;
	
	if (cut)
	{
		Helper::getMeshInformation(entity->getMesh(), vertexCount, vertices, indexCount, indices, node->getPosition(), node->getOrientation(), node->getScale());
		normals = new Ogre::Vector3[indexCount / 3];
		int index = 0;
		for (int i = 0; i < indexCount / 3; i += 3)
		{
			long index1 = indices[i];
			long index2 = indices[i + 1];
			long index3 = indices[i + 2];
			Ogre::Vector3 point1 = vertices[index1];
			Ogre::Vector3 point2 = vertices[index2];
			Ogre::Vector3 point3 = vertices[index3];
			Ogre::Vector3 currNormal = Helper::normalVector(point1, point2, point3);
			normals[index] = currNormal;
			index++;
		}

		createCopy();

		//RigidBody::cut(RigidBody::node->getPosition() - Ogre::Vector3(0,10,0), Ogre::Vector3(0.5f, 0.5f, 0));
	}
}

RigidBody::RigidBody(void)
{
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

bool RigidBody::hasFiniteMass()
{
	if (RigidBody::inverseMass == 0)
	{
		return false;
	}
	return true;
}

Ogre::Real RigidBody::getMass()
{
	return 1 / RigidBody::inverseMass;
}

Ogre::Real RigidBody::getInverseMass()
{
	return RigidBody::inverseMass;
}

void RigidBody::setInertiaTensor(const Ogre::Matrix3& inertiaTensor)
{
	RigidBody::inverseInertiaTensor = inertiaTensor.Inverse();
}



void sub3(const Ogre::Vector3* first, const Ogre::Vector3* second, Ogre::Vector3* result)
{
	result->x = first->x - second->x;
	result->y = first->y - second->y;
	result->z = first->z - second->z;
}

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

	Ogre::Vector3* leftVertices2 = new Ogre::Vector3[newVertexCount];
	Ogre::Vector3* leftNormals2 = new Ogre::Vector3[newVertexCount];
	int* leftIndices2 = new int[newIndexMax];
	int leftVertexCount2 = newVertexCount;
	int leftIndexCount2 = leftIndexCount;


	memcpy(leftVertices2, newVertices, leftVertexCount2 * sizeof(Ogre::Vector3));
	memcpy(leftNormals2, newNormals, leftVertexCount2 * sizeof(Ogre::Vector3));
	memcpy(leftIndices2, leftIndices, leftIndexCount2 * sizeof(int));

	//RigidBody::createColourCube();

	//right->vertices = new Vector3[newVertexCount];
	//right->vertexNormals = new Vector3[newVertexCount];
	//right->indices = new int[rightIndexCount];

	//right->vertexCount = newVertexCount;
	//right->indexCount = rightIndexCount;

	//memcpy(right->vertices, newVertices, right->vertexCount * sizeof(Vector3));
	//memcpy(right->vertexNormals, newNormals, right->vertexCount * sizeof(Vector3));
	//memcpy(right->indices, rightIndices, right->indexCount * sizeof(int));

	delete newVertices;
	delete newNormals;
	delete leftIndices;
	delete rightIndices;


	/*
	Ogre::Vector3* leftVertices2 = new Ogre::Vector3[newVertexCount];
	Ogre::Vector3* leftNormals2 = new Ogre::Vector3[newVertexCount];
	int* leftIndices2 = new int[newIndexMax];

	memcpy(leftVertices2, newVertices, newVertexCount);
	memcpy(leftNormals2, newNormals, newVertexCount);
	memcpy(leftIndices2, leftIndices, leftIndexCount * sizeof(int));


	const size_t vbufCount = 3 * newVertexCount;
	float* vertices = new float[vbufCount];
	
	for (size_t i = 0; i < newVertexCount; i++)
	{
		Ogre::Vector3 currVertex = leftVertices2[i];
		vertices[i * 3] = currVertex.x;
		vertices[i * 3 + 1] = currVertex.y;
		vertices[i * 3 + 2] = currVertex.z;
	}
	
	Ogre::Vector3* rightVertices2 = new Ogre::Vector3[newVertexCount];
	Ogre::Vector3* rightNormals2 = new Ogre::Vector3[newVertexCount];
	int* rightIndices2 = new int[newIndexMax];

	memcpy(rightVertices2, newVertices, newVertexCount);
	memcpy(rightNormals2, newNormals, newVertexCount);
	memcpy(rightIndices2, rightIndices, rightIndexCount * sizeof(int));
	


	delete leftVertices2;
	delete leftNormals2;
	delete rightVertices2;
	delete rightNormals2;
	delete leftIndices;
	delete leftIndices2;
	delete rightIndices;
	delete rightIndices2;
	*/
}



void RigidBody::createCopy()
{

	//gebasseerd op: https://www.grahamedgecombe.com/blog/2011/08/05/custom-meshes-in-ogre3d en http://www.ogre3d.org/tikiwiki/Generating+A+Mesh


	Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual("CustomMesh", "General");
	Ogre::SubMesh *subMesh = mesh->createSubMesh();

	/* create the vertex data structure */
	mesh->sharedVertexData = new Ogre::VertexData;
	mesh->sharedVertexData->vertexCount = vertexCount;

	/* declare how the vertices will be represented */
	Ogre::VertexDeclaration *decl = mesh->sharedVertexData->vertexDeclaration;
	size_t offset = 0;

	/* the first three floats of each vertex represent the position */
	decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
	offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

	/* create the vertex buffer */
	Ogre::HardwareVertexBufferSharedPtr vertexBuffer =
		Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
			offset, mesh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

	/* lock the buffer so we can get exclusive access to its data */
	
	float* _vertices = new float[vertexCount * 3];

	/* populate the buffer with some data */
	for (size_t i = 0; i < vertexCount; i++)
	{
		Ogre::Vector3 currVertex = vertices[i];
		_vertices[i * 3] = currVertex.x;
		_vertices[i * 3 + 1] = currVertex.y;
		_vertices[i * 3 + 2] = currVertex.z;
	}

	vertexBuffer->writeData(0, vertexBuffer->getSizeInBytes(), _vertices, true);
	Ogre::VertexBufferBinding* bind = mesh->sharedVertexData->vertexBufferBinding;
	bind->setBinding(0, vertexBuffer);

	/* create the index buffer */
	Ogre::HardwareIndexBufferSharedPtr indexBuffer = Ogre::HardwareBufferManager::getSingleton().
		createIndexBuffer(
			Ogre::HardwareIndexBuffer::IT_16BIT,
			mesh->sharedVertexData->vertexCount,
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);


	indexBuffer->writeData(0, indexBuffer->getSizeInBytes(), indices, true);


	/* attach the buffers to the mesh */
	mesh->sharedVertexData->vertexBufferBinding->setBinding(0, vertexBuffer);
	subMesh->useSharedVertices = true;
	subMesh->indexData->indexBuffer = indexBuffer;
	subMesh->indexData->indexCount = mesh->sharedVertexData->vertexCount;
	subMesh->indexData->indexStart = 0;

	/* set the bounds of the mesh */
	mesh->_setBounds(Ogre::AxisAlignedBox(-100, -100, -100, 100, 100, 100));

	/* notify the mesh that we're all ready */
	mesh->load();

	/* you can now create an entity/scene node based on your mesh, e.g. */
	Ogre::Entity *entity = mSceneMgr->createEntity("CustomEntity", "CustomMesh", "General");
	entity->setMaterialName("YourMaterial", "General");
	Ogre::SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	node->setPosition(100, 100, 100);
	node->attachObject(entity);

}

/*
/// Create the mesh via the MeshManager
Ogre::MeshPtr msh = Ogre::MeshManager::getSingleton().createManual("ColourCube", "General");

/// Create one submesh
Ogre::SubMesh* sub = msh->createSubMesh();


Ogre::RenderSystem* rs = Ogre::Root::getSingleton().getRenderSystem();



Ogre::RGBA* colours = new Ogre::RGBA[nVertices];
Ogre::RGBA *pColour = colours;
for (Ogre::uint32 i = 0; i < nVertices; i++)
{
rs->convertColourValue(Ogre::ColourValue(1.0, 1.0, 1.0), pColour++);
}

const size_t ibufCount = facesCount;

/// Create vertex data structure for 8 vertices shared between submeshes
msh->sharedVertexData = new Ogre::VertexData();
msh->sharedVertexData->vertexCount = nVertices;

/// Create declaration (memory format) of vertex data
Ogre::VertexDeclaration* decl = msh->sharedVertexData->vertexDeclaration;
size_t offset = 0;

decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

Ogre::HardwareVertexBufferSharedPtr vbuf =
Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
/// Upload the vertex data to the card
vbuf->writeData(0, vbuf->getSizeInBytes(), vertices, true);

/// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
Ogre::VertexBufferBinding* bind = msh->sharedVertexData->vertexBufferBinding;
bind->setBinding(0, vbuf);


/// Allocate index buffer of the requested number of vertices (ibufCount)
Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().
createIndexBuffer(
Ogre::HardwareIndexBuffer::IT_16BIT,
ibufCount,
Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

/// Upload the index data to the card
ibuf->writeData(0, ibuf->getSizeInBytes(), faces, true);

/// Set parameters of the submesh
sub->useSharedVertices = true;
sub->indexData->indexBuffer = ibuf;
sub->indexData->indexCount = ibufCount;
sub->indexData->indexStart = 0;

/// Set bounding information (for culling)
msh->_setBounds(Ogre::AxisAlignedBox(-100, -100, -100, 100, 100, 100));
msh->_setBoundingSphereRadius(sqrt(3 * 100 * 100));

/// Notify -Mesh object that it has been loaded
msh->load();

Ogre::MaterialPtr m_pMat = entity->getSubEntity(0)->getMaterial();
m_pMat->getTechnique(0)->getPass(0)->setAmbient(0, 1, 0);
m_pMat->getTechnique(0)->getPass(0)->setDiffuse(3, 20, 5, 20);


Ogre::Entity* thisEntity = mSceneMgr->createEntity("cc", "ColourCube");
thisEntity->setMaterialName(m_pMat->getName());
Ogre::SceneNode* thisSceneNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
thisSceneNode->setPosition(-35, 0, 0);
thisSceneNode->attachObject(thisEntity);

*/

