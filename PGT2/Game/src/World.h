#ifndef __World_h_
#define __World_h_

#include "BaseApplication.h"
#include "RigidBody.h"
#include "CustomCamera.h"
#include "InputManager.h"
#include "Helper.h"
#include "Ball.h"
#include "Building.h"
#include "Contact.h"
#include "MeshGenerator.h"



typedef struct
{
	Ogre::Vector2 minPoint;
	Ogre::Vector2 maxPoint;
	std::vector<RigidBody*> objects;
	size_t objectCount;
	std::vector<Face> terrainFaces;
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

	World();
	~World();
	
	Ogre::Light* directionalLight;
	CustomCamera camera;
	Ball* ballBody;
	RigidBody* groundBody;
	RigidBody* groundBody1;
	static Ogre::SceneManager* mSceneMgr;
	RigidBody* houseBody;

	static std::vector<RigidBody*> worldObjects;
	Ogre::Vector2 lowestMapPos;
	Ogre::Vector2 sectionSize;
	VerticeSection vertexSections[35][35];
	static size_t bodyCount;

	size_t terrainVertexCount, terrainIndexCount;
	Ogre::Vector3* terrainVertices;
	uint32_t* terrainIndices;

	void splitTerrainVertices();
	static void addRigidBody(RigidBody* body);
	void removeRigidBody(RigidBody* body);
	void addObjectVertices(RigidBody* body);
	virtual void createTerrain();
	virtual void createSphere();
	virtual void createLight();

	void createSphereMesh(const std::string& strName, const float r, const int nRings, const int nSegments);

	void createBuilding(Ogre::Vector3 pos);

	static RigidBody* createMesh(Ogre::Vector3* _verticesArr, int* _indicesArr, int _vertexCount, int _indexCount, Ogre::String matName);
	virtual void createHouse(Ogre::SceneManager* mSceneMgr);

	bool mouseMoved(const OIS::MouseEvent &arg);
	void createCamera(Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow);
	void createViewports();
	void restartWorld();
	void update(const Ogre::FrameEvent evt);
	void setCameraFollow();

	/** Holds the maximum number of contacts. */
	const static unsigned maxContacts = 256;

	/** Holds the array of contacts. */
	Contact contacts[maxContacts];

	/** Holds the collision data structure for collision detection. */
	CollisionData cData;

	/** Holds the contact resolver. */
	ContactResolver resolver;

	/** Processes the contact generation code. */
	void generateContacts();

	/** Processes the objects in the simulation forward in time. */
	void updateObjects(Ogre::Real duration);


private:
	
	void checkBallCollision();
	void addContact(CollisionData * data, Ogre::Vector3 contactNormal, Ogre::Vector3 contactPoint, Ogre::Real penetration, RigidBody * sphere);
	std::vector<Ogre::Vector2> getSections(Ogre::Vector3 pos, bool surround = false);
	std::vector<Ogre::Vector2> getSections(std::vector<Ogre::Vector3> positions);
	Ogre::Vector3 closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition);
	Ogre::Real clamp(Ogre::Real n, Ogre::Real lower, Ogre::Real upper);
	int clamp(int n, int lower, int upper);
};



#endif // #ifndef __World_h_
