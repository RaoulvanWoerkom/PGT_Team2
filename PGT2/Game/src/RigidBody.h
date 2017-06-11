#ifndef __RigidBody_h_
#define __RigidBody_h_


#include "BaseApplication.h"

struct Face
{
	Ogre::Vector3 point1;
	Ogre::Vector3 point2;
	Ogre::Vector3 point3;
	Ogre::Vector3 midPoint;

	Ogre::Vector3 normal;

} ;

class RigidBody 
{
protected:
	
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

	

	

	void loadMeshInfo();

public:

	Ogre::Real inverseMass;
	Ogre::SceneNode* node; //contains Position & Orientation
	Ogre::Entity* entity;
	std::string name;
	bool isDestroyed;
	bool isBreakable;
	bool canCollide;
	Ogre::Vector3 halfSize;
	std::vector<Ogre::Vector3> boundingBox;
	Ogre::Vector3* boundingBoxArray;
	//bool* cutList;
	//int cutListCount;


	RigidBody(Ogre::SceneNode* _node, Ogre::Entity* _entity);
	RigidBody(void);
	void setPosition(Ogre::Vector3 _position);
	void setVelocity(Ogre::Vector3 _velocity);
	Ogre::Vector3 getHalfsize();
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
	void createBoundingBox(float perc);
	Ogre::Vector3* getBoundingBox(bool worldPosition = true);
	bool hitBoxContainsPoint(Ogre::Vector3 point);
	bool setAndCheckIsAwake();

	void split(Ogre::Vector3 amount);
	void cut(Ogre::Vector3 planePoint, Ogre::Vector3 planeNormal);

	bool IsClockwise(Ogre::Vector3* newVertices, int index1, int index2, int index3);
	std::vector<int> fillIntersectionFaces(std::vector<int> _indices, std::vector<int> intersections, int middlePointIndex);
	
	size_t faceCount;
	std::vector<Face> faces;
	size_t vertexCount, indexCount;
	Ogre::Vector3* vertices;
	uint32_t* indices;
	Ogre::Vector3* normals;
	//void cut();
};

#endif // #ifndef __RigidBody_h_