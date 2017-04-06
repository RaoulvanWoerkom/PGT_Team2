#ifndef __World_h_
#define __World_h_

#include "BaseApplication.h"
#include "RigidBody.h"
#include "ForceGenerator.h"
#include "CustomCamera.h"
#include "InputManager.h"
#include "Helper.h"
#include "Ball.h"
#include "Contact.h"

typedef struct
{
	Ogre::Vector3 point1;
	Ogre::Vector3 point2;
	Ogre::Vector3 point3;

	Ogre::Vector3 normal;

} Vertex;

typedef struct
{
	Ogre::Vector2 minPoint;
	Ogre::Vector2 maxPoint;
	std::vector<Vertex> vertices;
} VerticeSection;

struct BodyRegistration
{
	RigidBody* body;
	BodyRegistration* next;
};

struct ContactGenRegistration
{
	ContactGenerator* gen;
	ContactGenRegistration* next;
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

class World
{


public:

	
	World(unsigned maxContacts, unsigned iterations = 0);
	World(void);
	~World();
	
	Ogre::Light* directionalLight;
	CustomCamera camera;
	Ball ballBody;
	RigidBody* groundBody;

	ForceRegistry registry;
	Gravity gravity;
	
	


	std::vector<RigidBody*> worldObjects;
	Ogre::Vector2 lowestMapPos;
	Ogre::Vector2 sectionSize;
	VerticeSection vertexSections[50][50];
	size_t bodyCount;

	size_t terrainVertexCount, terrainIndexCount;
	Ogre::Vector3* terrainVertices;
	unsigned long* terrainIndices;

	void splitTerrainVertices();
	void addRigidBody(RigidBody* body);
	void getMeshInformation(const Ogre::MeshPtr mesh,
		size_t &vertex_count,
		Ogre::Vector3* &vertices,
		size_t &index_count,
		unsigned long* &indices,
		const Ogre::Vector3 &position,
		const Ogre::Quaternion &orient,
		const Ogre::Vector3 &scale);
	virtual void createTerrain(Ogre::SceneManager* mSceneMgr);
	virtual void createSphere(Ogre::SceneManager* mSceneMgr);
	virtual void createLight(Ogre::SceneManager* mSceneMgr);
	bool mouseMoved(const OIS::MouseEvent &arg);
	void createCamera(Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow);
	void createViewports();
	void restartWorld();
	void update(const Ogre::FrameEvent evt);
	void setCameraFollow();

	ContactGenRegistration *firstContactGen;
	BodyRegistration *firstBody;
	ContactResolver resolver;
	bool calculateIterations;
	const static unsigned maxContacts = 256;
	Contact contacts[maxContacts];
	CollisionData cData;
	unsigned generateContacts();
	void startFrame();
	void runPhysics(Ogre::Real duration);



private:
	
	void checkBallCollision();
	std::vector<Ogre::Vector2> getSections(Ogre::Vector3 pos, bool surround = false);
	std::vector<Ogre::Vector2> getSections(std::vector<Ogre::Vector3> positions);
	Ogre::Vector3 closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition);
	float clamp(float n, float lower, float upper);
	int clamp(int n, int lower, int upper);
	Ogre::Vector3 normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3);
};



#endif // #ifndef __World_h_
