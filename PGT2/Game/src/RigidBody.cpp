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
	//cutList = new bool[vertexCount] {0};
	//cutListCount = vertexCount;
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
			face.midPoint = middlePoint;
			face.normal = Helper::normalVector(point1, point2, point3);
			faces.push_back(face);
			normals[faceCount] = face.normal;

			faceCount++;
		}
	}
}


void RigidBody::createBoundingBox()
{
	Ogre::Vector3 maxSize = Ogre::Vector3(10000000, 10000000, 10000000);
	Ogre::Vector3 minSize = Ogre::Vector3(-10000000, -10000000, -10000000);

	for (int i = 0; i < faces.size(); i++)
	{
		if (faces[i].midPoint.x < maxSize.x)
			maxSize.x = faces[i].midPoint.x;
		if (faces[i].midPoint.x > minSize.x)
			minSize.x = faces[i].midPoint.x;

		if (faces[i].midPoint.y < maxSize.y)
			maxSize.y = faces[i].midPoint.y;
		if (faces[i].midPoint.y > minSize.y)
			minSize.y = faces[i].midPoint.y;

		if (faces[i].midPoint.z < maxSize.z)
			maxSize.z = faces[i].midPoint.z;
		if (faces[i].midPoint.z > minSize.z)
			minSize.z = faces[i].midPoint.z;
	}

	boundingBox.push_back(maxSize);
	boundingBox.push_back(minSize);
	boundingBox.push_back(Ogre::Vector3(maxSize.x, maxSize.y, minSize.z));
	boundingBox.push_back(Ogre::Vector3(maxSize.x, minSize.y, maxSize.z));
	boundingBox.push_back(Ogre::Vector3(minSize.x, maxSize.y, maxSize.z));
	boundingBox.push_back(Ogre::Vector3(minSize.x, minSize.y, maxSize.z));
	boundingBox.push_back(Ogre::Vector3(minSize.x, maxSize.y, minSize.z));
	boundingBox.push_back(Ogre::Vector3(maxSize.x, minSize.y, minSize.z));
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
void RigidBody::cut(Ogre::Vector3 amount)
{
	srand(time(NULL));
	Ogre::Vector3 scale = node->getScale();
	Ogre::Vector3 cubeScale = scale / amount;
	for (size_t x = 0; x < amount.x; x++)
	{
		for (size_t y = 0; y < amount.y; y++)
		{
			for (size_t z = 0; z < amount.z; z++)
			{
				Ogre::Vector3 pos = getPosition() + Ogre::Vector3(x, y, z) * cubeScale * 100 - scale*50 + cubeScale * 50;


				Ogre::SceneNode* debrisNode = World::mSceneMgr->getRootSceneNode()->createChildSceneNode();
				debrisNode->setPosition(pos);
				debrisNode->setScale(cubeScale);
				int randNum = rand() % (1000000000);
				std::string meshName = "Cube" + std::to_string(randNum);
				Ogre::Entity* debrisEntity = World::mSceneMgr->createEntity("cube.mesh");

				debrisNode->attachObject(debrisEntity);
				RigidBody* debrisBody = new RigidBody(debrisNode, debrisEntity);
				debrisBody->entity->setMaterialName("Building/Wall");

				
				int randNum1 = rand() % (1000);
				int randNum2 = rand() % (1000);
				int randNum3 = rand() % (1000);
				Ogre::Vector3 ranDir = Ogre::Vector3(randNum1, randNum2, randNum3);
				ranDir.normalise();
				debrisBody->addForce(ranDir * 200);
				debrisBody->addRotation(-ranDir);



				//leftBody->canCollide = false;
				World::addRigidBody(debrisBody);

			}
		}
	}
	
}

/// \brief Fill in the missing faces on the intersection plane after slicing a mesh.
///
/// When slicing a mesh with the cut() method, the place where the meshes are cut is not yet filled with faces.
/// This results in a visual bug and collision issues.
/// This method will create new faces on the place where the mesh is cut, using the vertices that where created
/// in the cut method along the slicing plane edge. To create triangular faces from these intersection vertices,
/// indices are created between them and planePoint, because it is always exactly in the middle of all the other vertices.
std::vector<int> RigidBody::fillIntersectionFaces(std::vector<int> _indices, std::vector<int> intersections, int middlePointIndex)
{
	for (int i = 0; i < intersections.size(); i += 2)
	{
		_indices.push_back(intersections[i]);
		_indices.push_back(middlePointIndex);
		_indices.push_back(intersections[i + 1]);

		_indices.push_back(middlePointIndex);
		_indices.push_back(intersections[i]);
		_indices.push_back(intersections[i + 1]);
	}
	return _indices;
}





