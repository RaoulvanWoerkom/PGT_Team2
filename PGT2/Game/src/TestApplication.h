#ifndef __TestApplication_h_
#define __TestApplication_h_

#include "BaseApplication.h"

class TestApplication : public BaseApplication
{
public:
	TestApplication();
	virtual ~TestApplication();
	Ogre::SceneNode* ballNode;
	Ogre::SceneNode* groundNode;
	Ogre::Entity* ballEntity;
	Ogre::Entity* groundEntity;

protected:
	virtual void createScene();
	virtual void createCamera();
	virtual void createPlane();
	virtual void createLight();
	virtual void createViewports();
	virtual void createSphere();
	virtual void showScore(double score);
	float clamp(float n, float lower, float upper);
	void CheckBallCollision(Ogre::SceneNode* node1, Ogre::Entity* entity1);
	std::vector<Ogre::Vector3>  FindLineSphereIntersections(Ogre::Vector3 linePoint0, Ogre::Vector3 linePoint1, Ogre::Vector3 circleCenter, double circleRadius);
	void GetMeshInformation(const Ogre::MeshPtr mesh,
		size_t &vertex_count,
		Ogre::Vector3* &vertices,
		size_t &index_count,
		unsigned long* &indices,
		const Ogre::Vector3 &position,
		const Ogre::Quaternion &orient,
		const Ogre::Vector3 &scale);
	virtual Ogre::Vector3 closestPointOnTriangle(Ogre::Vector3 point1, Ogre::Vector3 point2, Ogre::Vector3 point3, const Ogre::Vector3 &sourcePosition );

	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);
	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);
};


#endif // #ifndef __TestApplication_h_

