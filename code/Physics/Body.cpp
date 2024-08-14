//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
m_position( 0.0f ),
m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
m_linearVelocity( 0.0f ),
m_invMass( 1.0f ),
m_elasticity( 1.0f ),
m_shape( NULL ) {
}

Vec3 Body::GetCenterOfMassWorldSpace() const{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
	return pos;
}

Vec3 Body::GetCenterOfMassModelSpace() const {
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}

Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPt) const {
	Vec3 tmp = worldPt - GetCenterOfMassWorldSpace();
	Quat inverseOrient = m_orientation.Inverse();
	Vec3 bodySpace = inverseOrient.RotatePoint(tmp);
	return bodySpace;
}

Vec3 Body::BodySpaceToWorldSpace(const Vec3& worldPt) const {
	Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(worldPt);
	return worldSpace;
}


void Body::ApplyImpulseLinear(const Vec3& impulse) {
	if (0.0f == m_invMass) {
		return;
	}
	// p = mv
	// dp = F * dt
	// J == dp = F * dt
	// dv = dp / m
	// dv = J / m

	m_linearVelocity += impulse * m_invMass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse) {
	if (0.0f == m_invMass) {
		return;
	}

	// L = Iw = r x p
	// dL = I * dw = r x dp = r x J
	// dw = I^-1 * (r x J)
	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * impulse;

	const float max_angular_velocity = 30.0f; // arbitrary fast enough value
	if (m_angularVelocity.GetLengthSqr() > pow(max_angular_velocity, 2))
	{
		m_angularVelocity.Normalize();
		m_angularVelocity *= max_angular_velocity;
	}
}



Mat3 Body::GetInverseInertiaTensorBodySpace() const {
	Mat3 inertia_tensor = m_shape->InertiaTensor();
	// I^-1 * m^-1 
	Mat3 inverse_inertia_tensor = m_shape->InertiaTensor().Inverse() *  m_invMass;
	return inverse_inertia_tensor;
}

Mat3 Body::GetInverseInertiaTensorWorldSpace() const {
	Mat3 inertia_tensor = m_shape->InertiaTensor();
	Mat3 inverse_inertia_tensor = m_shape->InertiaTensor().Inverse() * m_invMass;
	Mat3 orient = m_orientation.ToMat3();
	inverse_inertia_tensor = orient * inverse_inertia_tensor * orient.Transpose();
	return inverse_inertia_tensor;
}

