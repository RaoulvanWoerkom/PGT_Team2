#ifndef __RigidBody_h_
#define __RigidBody_h_


#include "BaseApplication.h"

struct Face
{
	Ogre::Vector3 point1;
	Ogre::Vector3 point2;
	Ogre::Vector3 point3;

	Ogre::Vector3 normal;

} ;

class RigidBody 
{
protected:
	Ogre::Real inverseMass;
	Ogre::Real dampening;
	Ogre::Vector3 forceAccum;
	Ogre::Vector3 torqueAccum;
	Ogre::Vector3 velocity;
	Ogre::Vector3 rotation;
	Ogre::Vector3 acceleration;
	Ogre::Vector3 lastFrameAcceleration;

	Ogre::Matrix4 transformMatrix;
	Ogre::Matrix3 inverseInertiaTensorWorld;
	Ogre::Matrix3 inverseInertiaTensor;
	Ogre::Matrix3 inertiaTensor;
	Ogre::SceneManager* mSceneMgr;

	bool isAwake;
	bool canSleep;



	size_t vertexCount, indexCount;
	Ogre::Vector3* vertices;
	uint32_t* indices;
	Ogre::Vector3* normals;

	void loadMeshInfo();

public:
	Ogre::SceneNode* node; //contains Position & Orientation
	Ogre::Entity* entity;

	bool isBreakable;
	bool canCollide;

	RigidBody(Ogre::SceneNode* _node, Ogre::Entity* _entity);
	RigidBody(void);
	void setPosition(Ogre::Vector3 _position);
	void setVelocity(Ogre::Vector3 _velocity);
	Ogre::Vector3 getVelocity();
	Ogre::Vector3 getRotation();
	Ogre::Vector3 getPosition();
	Ogre::Vector3 getLastFrameAcceleration();
	Ogre::Matrix3 getInverseInertiaTensorWorld();
	void addVelocity(Ogre::Vector3 vel);
	void addRotation(Ogre::Vector3 vel);
	void setOrientation(Ogre::Quaternion& _orientation);
	void setInertiaTensor(const Ogre::Matrix3& _inertiaTensor);
	Ogre::Quaternion getOrientation();
	void addForce(Ogre::Vector3 _force);
	void addTorque(Ogre::Vector3 _torque);
	void addForceAtBodyPoint(Ogre::Vector3 force, Ogre::Vector3 point);
	void addForceAtPoint(Ogre::Vector3 force, Ogre::Vector3 point);
	void setIsAwake(const bool awake);
	void _transformInertiaTensor(Ogre::Matrix3 & iitWorld, Ogre::Quaternion q, Ogre::Matrix3 & iitBody, Ogre::Matrix4 & rotmat);
	void clearAccumulators();
	void calculateDerivedData();
	virtual void integrate(Ogre::Real delta);
	bool hasFiniteMass();
	Ogre::Real getMass();
	Ogre::Real getInverseMass();

	void cut(Ogre::Vector3 planePoint, Ogre::Vector3 planeNormal);
	bool IsClockwise(Ogre::Vector3* newVertices, int index1, int index2, int index3);
	void fillIntersectionFaces(Ogre::Vector3* _verticesArr, int* _indicesArr, int _vertexCount, int _indexCount, int intersectionTotal, int* intersectionsArray, Ogre::Vector3* newVertices, int middlePointIndex);
	
	size_t faceCount;
	std::vector<Face> faces;

	//void cut();
};

#endif // #ifndef __RigidBody_h_