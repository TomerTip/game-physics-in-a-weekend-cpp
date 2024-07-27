//
//  Scene.h
//
#pragma once
#include <vector>
#include <iostream>

#include "Physics/Shapes.h"
#include "Physics/Body.h"
#include "Physics/Constraints.h"
#include "Physics/Manifold.h"

/*
====================================================
Scene
====================================================
*/


class Scene {
public:
	Scene() { m_bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	std::vector< Body > m_bodies;
	std::vector< Constraint * >	m_constraints;
	ManifoldCollector m_manifolds;


	typedef struct contact_t {
		Vec3 ptr_on_A_worldspace;
		Vec3 ptr_on_B_worldspace;
		Vec3 ptr_on_A_localspace;
		Vec3 ptr_on_B_localspace;

		Vec3 normal; // In world space coordinates
		float separation_distance; // Positive value when non-penetrating, Negative value when penetrating
		float time_of_impact;

		Body* bodyA;
		Body* bodyB;
	} contact_t;


	void ResolveContact(contact_t &contact);
	bool Intersect(Body* a, Body* b, contact_t &contact);
};
