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

bool Scene::Intersect(const Body* a, const Body* b) {
	const Vec3 ab = b->m_position - a->m_position;
	const ShapeSphere* sphere_a = (const ShapeSphere*)a->m_shape;
	const ShapeSphere* sphere_b = (const ShapeSphere*)b->m_shape;

	const float radius_ab = sphere_a->m_radius + sphere_b->m_radius;
	const float length_sqr = ab.GetLengthSqr();
	
	// a,b -  radii, d - length of distance
	// if a + b > d, intersect.
	return (pow(radius_ab, 2) < length_sqr) ? false : true;
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec ) {
	for (size_t i = 0; i < m_bodies.size(); i++)
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
	}

	// Collision Check
	for (size_t i = 0; i < m_bodies.size(); i++)
	{
		for (size_t j = i + 1; j < m_bodies.size(); j++)
		{
			Body* a = &m_bodies[i];
			Body* b = &m_bodies[j];

			// Skip bodies with infinite mass
			if (a->m_invMass == 0.0f && b->m_invMass == 0.0f)
			{
				continue;
			}

			if (Intersect(a, b))
			{
				std::cout << "intersect" << std::endl;
				a->m_linearVelocity.Zero();
				b->m_linearVelocity.Zero();
			}
		}
	}

	for (size_t i = 0; i < m_bodies.size(); i++)
	{	
		// Skip ground
		if (i != 0)		
		{
			// dx = v * dt
			m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
		}
	}
}