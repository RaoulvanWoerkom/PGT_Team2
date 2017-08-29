#ifndef __SingleParticle_h_
#define __SingleParticle_h_
#include "BaseApplication.h"
#include "Ogre.h"
#include "Baller.h"

	class SingleParticle
	{
	public:
		SingleParticle(Ogre::Vector3 tVelocity, int tDuration, Ogre::SceneNode * node);
		SingleParticle(void);
		bool Update();
		Ogre::Vector3 getPosition() {
			return position;
		}
		Ogre::Vector3 getVelocity() {
			return velocity;
		}
		int getDuration() {
			return pastDuration;
		}
		Ogre::ColourValue getColor() {
			return color;
		}
		static Ogre::SceneManager* mSceneManager;
	private:
		Ogre::Vector3 position;
		Ogre::Vector3 velocity;
		int duration;
		int pastDuration;
		Ogre::ColourValue color;
		void DrawParticle(Ogre::SceneNode * node);
		Ogre::SceneNode* particleNode;
		Ogre::Entity* particle;
	};

#endif

