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
	body.m_position = Vec3( 0, 0, -1001 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere(1000);
	body.m_invMass = 0; // "Infinite mass" - causes gravity to not have any effect.
	body.m_elasticity = 1.0f;
	body.m_friction = 1.0f;
	m_bodies.push_back( body );
	
	for (size_t i = 1; i < 20; i++) {
		float radius = 0.1 * i;

		body.m_position = Vec3(-5, -5 + i + (i * radius), 2 * (i + radius));  // Position the spheres in a circle
		body.m_orientation = Quat(0, 0, 0, 1);
		body.m_shape = new ShapeSphere(radius);
		body.m_invMass = 1.0f / 0.5f;  // mass of 2 kg
		body.m_elasticity = 1;
		body.m_friction = 0.2f;
		body.m_linearVelocity = Vec3(0, 1, 0);

		m_bodies.push_back(body);
	}


	// TODO: Add code
}

void Scene::ResolveContact(contact_t &contact) {
	Body* a = contact.bodyA;
	Body* b = contact.bodyB;

	const Vec3 point_on_a = contact.point_on_A_worldspace;
	const Vec3 point_on_b = contact.point_on_B_worldspace;

	const float elasticity_a = a->m_elasticity;
	const float elasticity_b = b->m_elasticity;
	const float elasticity = elasticity_a * elasticity_b;

	const float invMass_a = a->m_invMass;
	const float invMass_b = b->m_invMass;

	const Mat3 invWorldInertia_a = a->GetInverseInertiaTensorWorldSpace();
	const Mat3 invWorldInertia_b = b->GetInverseInertiaTensorWorldSpace();

	const Vec3& n = contact.normal;

	const Vec3 radius_a = point_on_a - a->GetCenterOfMassWorldSpace();
	const Vec3 radius_b = point_on_b - b->GetCenterOfMassWorldSpace();

	const Vec3 angular_impulse_a = (invWorldInertia_a * radius_a.Cross(n)).Cross(radius_a);
	const Vec3 angular_impulse_b = (invWorldInertia_b * radius_b.Cross(n)).Cross(radius_b);
	const float angular_factor = (angular_impulse_a + angular_impulse_b).Dot(n);

	// Get the world space velocity of motion and rotation
	const Vec3 vel_a = a->m_linearVelocity + a->m_angularVelocity.Cross(radius_a);
	const Vec3 vel_b = b->m_linearVelocity + b->m_angularVelocity.Cross(radius_b);

	// Calculate the collision impulse
	const Vec3 vab = vel_a - vel_b;
	const float impulse = (1.0f + elasticity) * vab.Dot(n) / (invMass_a + invMass_b + angular_factor);
	const Vec3 impulse_vector = n * impulse;

	a->ApplyImpulse(point_on_a, impulse_vector * -1.0f);
	b->ApplyImpulse(point_on_b, impulse_vector * 1.0f);

	// Calculate impulse caused by friction
	const float friction_a = a->m_friction;
	const float friction_b = b->m_friction;
	const float friction = friction_a * friction_b;

	// Find the normal direction of the velocity in respect with the normal of the collision
	const Vec3 velocity_normal = n * n.Dot(vab);

	// Find the tangent dirction of the velocity in respect with the normal of the collision
	const Vec3 velocity_tangent = vab - velocity_normal;

	Vec3 relative_tangent_velocity = velocity_tangent;
	relative_tangent_velocity.Normalize();

	const Vec3 inertia_a = (invWorldInertia_a * radius_a.Cross(relative_tangent_velocity)).Cross(radius_a);
	const Vec3 inertia_b = (invWorldInertia_b * radius_b.Cross(relative_tangent_velocity)).Cross(radius_b);
	const float inv_inertia = (inertia_a + inertia_b).Dot(relative_tangent_velocity);

	// Calculate the tangential impulse for friction
	const float reduced_mass = 1.0f / (invMass_a + invMass_b + inv_inertia);
	const Vec3 impulse_friction = velocity_tangent * reduced_mass * friction;

	a->ApplyImpulse(point_on_a, impulse_friction * -1.0f);
	b->ApplyImpulse(point_on_b, impulse_friction * 1.0f);


	// Moving by new center of mass
	
	// TODO: Buggy, understand why
	
	//const float tA = invMass_a / (invMass_a + invMass_b);
	//const float tB = invMass_b / (invMass_b + invMass_a);
	//const Vec3 distance = contact.point_on_B_localspace - contact.point_on_A_worldspace;
	//a->m_position += distance * tA;
	//b->m_position -= distance * tB;
}

bool Scene::Intersect(Body* a, Body* b, contact_t &contact) {
	contact.bodyA = a;
	contact.bodyB = b;

	const Vec3 ab = b->m_position - a->m_position;
	const ShapeSphere* sphere_a = (const ShapeSphere*)a->m_shape;
	const ShapeSphere* sphere_b = (const ShapeSphere*)b->m_shape;

	contact.normal = ab;
	contact.normal.Normalize();

	contact.point_on_A_worldspace = a->m_position + (contact.normal * sphere_a->m_radius);
	contact.point_on_B_worldspace = b->m_position - (contact.normal * sphere_b->m_radius);

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

			contact_t contact;
			if (Intersect(a, b, contact))
			{
				ResolveContact(contact);
			}
		}
	}

	// Update position
	for (size_t i = 0; i < m_bodies.size(); i++)
	{	
		m_bodies[i].Update(dt_sec);
	}
}