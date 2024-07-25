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
			// Gravitational Acceleration
			// dv = a * dt
			// g = 10 m/s
			float gravity = -10.0f;
			m_bodies[i].m_linearVelocity += Vec3(0, 0, gravity) * dt_sec;
		}
	}

	for (size_t i = 0; i < m_bodies.size(); i++)
	{
		// Except Ground
		if (i != 0)
		{
			// Position
			// dx = v * dt
			m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
		}
	}
}