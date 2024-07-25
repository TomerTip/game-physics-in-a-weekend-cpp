//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;

	// World Sphere
	body.m_position = Vec3( 0, 0, -101 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 100.0f );
	m_bodies.push_back( body );

	body.m_position = Vec3( 0, 0, 10 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 2.0f );
	m_bodies.push_back( body );


	// TODO: Add code
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	for (size_t i = 0; i < m_bodies.size(); i++)
	{
		// Except Ground
		if (i != 0)
		{
			Body* body = &m_bodies[i];

			// Gravity as an impulse
			// J = dp
			// F = dp/dt => dp = F * dt => J = F * dt
			// F = mg * dt = > J = mg * dt ^ 2

			// g = 9.8 m/s
			float gravity = -9.8f;

			float mass = 1.0f / body->m_invMass;
			Vec3 impulseGravity = Vec3(0, 0, gravity) * mass * dt_sec;
			body->ApplyImpulseLinear(impulseGravity);
			
			// dx = v * dt
			m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
		}
	}
}