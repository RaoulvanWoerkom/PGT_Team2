#ifndef __ParticleSystemBase_h_
#define __ParticleSystemBase_h_
#include <iostream>
#include <vector>
#include "BaseApplication.h"
#include "OgreSceneManager.h"
#include "SingleParticle.h"
class ParticleSystemBase
{
public:
	ParticleSystemBase(Ogre::SceneNode * node);
	ParticleSystemBase(void);
	bool Update();
	void Draw();

protected:
	std::vector<SingleParticle> particles;
	bool regenerate;
	Ogre::Vector3 particleVelocity;
	Ogre::ColourValue color;
	SingleParticle GenerateParticle();
	Ogre::SceneNode* partSystemNode;
	Ogre::SceneNode* tempPartNode;
	
private:
};
#endif 

