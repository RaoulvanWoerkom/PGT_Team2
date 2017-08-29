#include "ParticleSystemBase.h"


int DEF_NUM_PARTICLES = 300;
ParticleSystemBase::ParticleSystemBase(Ogre::SceneNode* node)
{	
	particles.reserve(DEF_NUM_PARTICLES);
	partSystemNode = node;
	for (int i = 0; i < DEF_NUM_PARTICLES; i++)
	{
		particles.emplace_back(GenerateParticle());
	}
}


ParticleSystemBase::ParticleSystemBase(void)
{
}

bool ParticleSystemBase::Update() 
{
	SingleParticle part;
	int partCount = DEF_NUM_PARTICLES;
	for (int i = 0; i < partCount; i++)
	{
		part = particles[i];
		if (!part.Update())
		{
			particles.erase(particles.begin() + i);
			i--;
			partCount--;
		}
	}
	if (partCount <= 0) {
		return false;
	}
	else {
		return true;
	}
}
//void ParticleSystemBase::Draw()
//{
// instead of using the system to draw all the particles i made it so the particles draw themselves ones they are spawned;
//}
SingleParticle ParticleSystemBase::GenerateParticle()
{	
	int duration = Ogre::Math::RangeRandom(5, 10);
	tempPartNode = partSystemNode->createChildSceneNode();
	particleVelocity = Ogre::Vector3(Ogre::Math::RangeRandom(1, 10), Ogre::Math::RangeRandom(1, 10), Ogre::Math::RangeRandom(1, 10));
	SingleParticle part = SingleParticle(particleVelocity, duration, tempPartNode);
	return part;
}
