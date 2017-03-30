#ifndef __World_h_
#define __World_h_

#include "BaseApplication.h"
#include "RigidBody.h"
#include "ForceGenerator.h"
#include "CustomCamera.h"
#include "InputManager.h"
#include "Helper.h"

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



class World
{


public:

	
	World();
	
	Ogre::Light* directionalLight;
	CustomCamera camera;
	RigidBody ballBody;
	RigidBody* groundBody;

	ForceRegistry registry;
	Gravity gravity;
	


	std::vector<RigidBody*> worldObjects;
	Ogre::Vector2 lowestMapPos;
	Ogre::Vector2 sectionSize;
	VerticeSection vertexSections[20][20];
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
	void update(const Ogre::FrameEvent& evt);
	void setCameraFollow();

	

private:
	
	void checkBallCollision();
	std::vector<Ogre::Vector2> getSections(Ogre::Vector3 pos, Ogre::Vector2 size);
	std::vector<Ogre::Vector2> getSections(std::vector<Ogre::Vector3> positions);
	Ogre::Vector3 closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition);
	float clamp(float n, float lower, float upper);
	int clamp(int n, int lower, int upper);
	Ogre::Vector3 normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3);
};



#endif // #ifndef __World_h_
