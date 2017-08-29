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
#include "CollisionDetector.h"
#include "ParticleSystemBase.h"
#include "SingleParticle.h"



struct VerticeSection
{
	Ogre::Vector2 minPoint;
	Ogre::Vector2 maxPoint;
	std::vector<CollisionBox*>* objects;
	std::vector<Face> terrainFaces;
};

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


class World
{
public:

	World();
	~World();
	
	int* currentScore;

	Ogre::Light* directionalLight;
	CustomCamera camera;
	Ball* ballBody;
	RigidBody* groundBody;
	static Ogre::SceneManager* mSceneMgr;
	RigidBody* houseBody;

	float jumpPower;

	static const int maxDebris = 70;

	static std::vector<CollisionBox*> World::worldObjects;
	static std::queue<RigidBody*> World::debrisQueue;
	Ogre::Vector2 lowestMapPos;
	Ogre::Vector2 sectionSize;
	VerticeSection* vertexSections[25][25];
	static size_t boxCount;



	size_t terrainVertexCount, terrainIndexCount;
	Ogre::Vector3* terrainVertices;
	uint32_t* terrainIndices;



	void splitTerrainVertices();
	static void addCollisionBox(CollisionBox* box);
	virtual void createTerrain();
	virtual void createSphere();
	virtual void createLight();

	void createSphereMesh(const std::string& strName, const float r, const int nRings, const int nSegments);

	void createBuilding(Ogre::Vector3 pos);

	static Ogre::Entity* createCustomEntity(Ogre::Vector3* _verticesArr, std::vector<int> _indiceList, int _vertexCount, Ogre::String matName);
	static void addDebris(RigidBody * body);

	virtual void createHouse(Ogre::SceneManager* mSceneMgr);

	bool mouseMoved(const OIS::MouseEvent &arg);
	void createCamera(Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow);
	void createViewports();
	void restartWorld();
	void update(const Ogre::FrameEvent evt);
	void setCameraFollow();
	void initializeSections();
	void emptySectionObjects();
	void populateSections();
	void createParticleSystem(Ogre::SceneNode* node);

	Ogre::SceneNode* particleNode;
	ParticleSystemBase particleSystem;
	

	/** Holds the maximum number of contacts. */
	const static unsigned maxContacts = 900;

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
	void checkWorldCollision();
	void addBallContact(CollisionData * data, Ogre::Vector3 contactNormal, Ogre::Vector3 contactPoint, Ogre::Real penetration, RigidBody* sphere);
	std::vector<Ogre::Vector2> getSections(Ogre::Vector3 pos, bool surround = false);
	std::vector<Ogre::Vector2> getSections(Ogre::Vector3* positions, int size);
	std::vector<Ogre::Vector2> getSections(std::vector<Ogre::Vector3> positions);
	Ogre::Vector3 closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition);
	Ogre::Real clamp(Ogre::Real n, Ogre::Real lower, Ogre::Real upper);
	int clamp(int n, int lower, int upper);
};



#endif // #ifndef __World_h_
