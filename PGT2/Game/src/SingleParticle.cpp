#include "SingleParticle.h"
#include "Ogre.h"

// Made with help from https://www.codeproject.com/Articles/10003/A-basic-Particles-System
// http://buildnewgames.com/particle-systems/
// http://natureofcode.com/book/chapter-4-particle-systems/
// tested with help from https://www.youtube.com/watch?v=8vB-MP5-82U

const float DRAG = 0.9;
const Ogre::Vector3 GRAVITY = Ogre::Vector3(0,-9.81,0);
Ogre::SceneManager* SingleParticle::mSceneManager = NULL;

SingleParticle::SingleParticle(Ogre::Vector3 tVelocity, int tDuration, Ogre::SceneNode* node)
{
	velocity = tVelocity;
	duration = tDuration;
	particleNode = node;
	DrawParticle(node);
	particleNode->setPosition(position);
}


SingleParticle::SingleParticle(void)
{
}

bool SingleParticle::Update() 
{
	position = particleNode->getPosition();
	velocity = velocity * Ogre::Vector3(DRAG, 0, DRAG);
	velocity = velocity + GRAVITY;
	position = position + velocity;
	particleNode->setPosition(position);
	// move the scenenode made in the draw function that i am gonna put in this class ;)
	pastDuration++;
	if (pastDuration > duration)
	{
		return false;
	}
	else {
		return true;
	}
}

void SingleParticle::DrawParticle(Ogre::SceneNode * node)
{
	particle = mSceneManager->createEntity("Particle", Ogre::SceneManager::PT_SPHERE);
	node->attachObject(particle);
	particle->setMaterialName("Particle/Color");
}
