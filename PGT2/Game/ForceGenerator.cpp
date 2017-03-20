#include "ForceGenerator.h"

void ForceRegistry::add(RigidBody * body, ForceGenerator * fg)
{
	ForceRegistry::ForceRegistration registration;
	registration.body = body;
	registration.fg = fg;
	registrations.push_back(registration);
}

void ForceRegistry::remove(RigidBody * body, ForceGenerator * fg)
{
}

void ForceRegistry::clear()
{
}

void ForceRegistry::updateForces(float duration)
{
	Registry::iterator i = registrations.begin();
	for (; i != registrations.end(); i++)
	{
		i->fg->updateForce(i->body, duration);
	}
}

//Gravity 
Gravity::Gravity(const Ogre::Vector3 & gravity)
{
	Gravity::gravity = gravity;
}

void Gravity::updateForce(RigidBody * body, float duration)
{
	if (!body->HasFiniteMass) return;

	// Apply the mass-scaled force to the body
	body->AddForce(gravity * body->GetMass());
}