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

void Body::ApplyImpulse(const Vec3& impulse_point, const Vec3& impulse) {
	/*
		impulse_point is the position of the impulse in world space.
		impulse is the direction and the magnitude of the impulse.
	*/
	if (0.0f == m_invMass)
	{
		return;
	}

	const Vec3& linear_impulse = impulse;
	ApplyImpulseLinear(linear_impulse);

	Vec3 body_position = GetCenterOfMassWorldSpace();
	
	// r - radius(distance from center of mass to impulse point) 
	// angular_impulse = r x linear_impulse
	Vec3 r = impulse_point - body_position;
	Vec3 angular_impulse = r.Cross(impulse);
	ApplyImpulseAngular(angular_impulse);
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

void Body::Update(const float dt_sec) {
	// Update Position:
	// dx = v * dt
	m_position += m_linearVelocity * dt_sec;

	Vec3 pos_center_of_mass = GetCenterOfMassWorldSpace();
	Vec3 center_of_mass_to_pos = m_position - pos_center_of_mass;

	Mat3 orientation = m_orientation.ToMat3();
	Mat3 inertia_tensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();

	// a = I^-1 * (w x I * w)
	Vec3 alpha = inertia_tensor.Inverse() * (m_angularVelocity.Cross(inertia_tensor * m_angularVelocity));

	// w = a * dt
	m_angularVelocity += alpha * dt_sec;

	// Update Orientation:
	// d_theta = w * dt
	Vec3 delta_angle = m_angularVelocity * dt_sec;
	Quat dq = Quat(delta_angle, delta_angle.GetMagnitude());
	m_orientation = dq * m_orientation;
	m_orientation.Normalize();

	// Update new model position
	m_position = pos_center_of_mass + dq.RotatePoint(center_of_mass_to_pos);
}
