#ifndef __World_h_
#define __World_h_

#include "BaseApplication.h"
#include "RigidBody.h"
#include "ForceGenerator.h"
#include "CustomCamera.h"
#include "InputManager.h"

class World
{


public:


	World();
	Ogre::Light* directionalLight;
	CustomCamera camera;
	RigidBody ballBody;
	Ogre::SceneNode* groundNode;
	Ogre::Entity* groundEntity;
	ForceRegistry registry;
	Gravity gravity;

	
	

	std::vector<RigidBody*> worldObjects;
	size_t bodyCount;

	size_t vertex_count, index_count;
	Ogre::Vector3* vertices;
	unsigned long* indices;

	void splitVertices();
	void addRigidBody(RigidBody* body);
	void getMeshInformation(const Ogre::MeshPtr mesh,
		size_t &vertex_count,
		Ogre::Vector3* &vertices,
		size_t &index_count,
		unsigned long* &indices,
		const Ogre::Vector3 &position,
		const Ogre::Quaternion &orient,
		const Ogre::Vector3 &scale);
	virtual void createPlane(Ogre::SceneManager* mSceneMgr);
	virtual void createSphere(Ogre::SceneManager* mSceneMgr);
	virtual void createLight(Ogre::SceneManager* mSceneMgr);
	bool mouseMoved(const OIS::MouseEvent &arg);
	void createCamera(Ogre::Camera* mCamera, Ogre::SceneManager* mSceneMgr, Ogre::RenderWindow* mWindow);
	void createViewports();
	void restartWorld();
	void update(const Ogre::FrameEvent& evt);
	void setCameraFollow();

private:
	
	void checkBallCollision(Ogre::Entity* otherEntity, Ogre::SceneNode* otherSceneNode);
	Ogre::Vector3 closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition);
	float clamp(float n, float lower, float upper);
	Ogre::Vector3 normalVector(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3);
};


#endif // #ifndef __World_h_
