#include "AntiRollContraints.h"
#include "PsMathUtils.h"
#include "foundation/PxMath.h"
#include "vehicle/PxVehicleDrive4W.h"
#include "PxScene.h"
#include "PxPhysics.h"
#include "../SnippetVehicleCommon/SnippetVehicleCreate.h"

namespace physx
{
	extern PxVec3 gRight;
	extern PxVec3 gUp;
	extern PxVec3 gForward;
}

using namespace physx;

PxVec3 projection(PxVec3 v1, PxVec3 v2)
{
	return v2 * (v1.dot(v2));
}

/**
   Decompose the rotation on to 2 parts.
   1. Twist - rotation around the "direction" vector
   2. Swing - rotation around axis that is perpendicular to "direction" vector
   The rotation can be composed back by
   rotation = swing * twist

   has singularity in case of swing_rotation close to 180 degrees rotation.
   if the input quaternion is of non-unit length, the outputs are non-unit as well
   otherwise, outputs are both unit
*/
void swing_twist_decomposition(const PxQuat& rotation,
	const PxVec3& direction,
	PxQuat& twist)
{
	PxVec3 ra(rotation.x, rotation.y, rotation.z); // rotation axis
	PxVec3 p = projection(ra, direction); // return projection v1 on to v2  (parallel component)
	twist = PxQuat(p.x, p.y, p.z, rotation.w);
	twist.normalize();
}

PX_INLINE void computeJacobianAxes(PxVec3 row[3], const PxQuat& qa, const PxQuat& qb)
{
	// Compute jacobian matrix for (qa* qb)  [[* means conjugate in this expr]]
	// d/dt (qa* qb) = 1/2 L(qa*) R(qb) (omega_b - omega_a)
	// result is L(qa*) R(qb), where L(q) and R(q) are left/right q multiply matrix

	PxReal wa = qa.w, wb = qb.w;
	const PxVec3 va(qa.x, qa.y, qa.z), vb(qb.x, qb.y, qb.z);

	const PxVec3 c = vb * wa + va * wb;
	const PxReal d0 = wa * wb;
	const PxReal d1 = va.dot(vb);
	const PxReal d = d0 - d1;

	row[0] = (va * vb.x + vb * va.x + PxVec3(d, c.z, -c.y)) * 0.5f;
	row[1] = (va * vb.y + vb * va.y + PxVec3(-c.z, d, c.x)) * 0.5f;
	row[2] = (va * vb.z + vb * va.z + PxVec3(c.y, -c.x, d)) * 0.5f;

	if ((d0 + d1) != 0.0f)  // check if relative rotation is 180 degrees which can lead to singular matrix
		return;
	else
	{
		row[0].x += PX_EPS_F32;
		row[1].y += PX_EPS_F32;
		row[2].z += PX_EPS_F32;
	}
}

PX_FORCE_INLINE Px1DConstraint* angular(const PxVec3& axis, PxReal posErr, Px1DConstraint* c, bool soft)
{
	c->linear0 = PxVec3(0.f);		c->angular0 = axis;
	c->linear1 = PxVec3(0.f);		c->angular1 = axis;

	c->geometricError = posErr;
	if (soft)
	{
		c->solveHint = PxU16(PxConstraintSolveHint::eNONE);
		c->mods.spring.stiffness = 80;
		c->mods.spring.damping = 10;
		c->flags = Px1DConstraintFlag::eSPRING | Px1DConstraintFlag::eACCELERATION_SPRING;
	}
	else
	{
		c->solveHint = PxU16(PxConstraintSolveHint::eEQUALITY);
		c->flags = 0;
	}
	return c;
}

PX_FORCE_INLINE Px1DConstraint* angularVelocity(const PxVec3& axis, PxReal velocityTarget, Px1DConstraint* c, bool soft)
{
	c->linear0 = PxVec3(0.f);		c->angular0 = axis;
	c->linear1 = PxVec3(0.f);		c->angular1 = axis;

	c->geometricError = 0;
	c->velocityTarget = velocityTarget;
	if (soft)
	{
		c->solveHint = PxU16(PxConstraintSolveHint::eNONE);
		c->mods.spring.stiffness = 80;
		c->mods.spring.damping = 10;
		c->flags = Px1DConstraintFlag::eSPRING | Px1DConstraintFlag::eACCELERATION_SPRING;
	}
	else
	{
		c->solveHint = PxU16(PxConstraintSolveHint::eEQUALITY);
		c->flags = 0;
	}
	return c;
}

PxU32 AntiRollContraints::antiRollConstraintSolverPrep(Px1DConstraint* c,
    PxVec3& body0WorldOffset, 
    PxU32 maxConstraints,
    PxConstraintInvMassScale&, 
    const void* constantBlock, 
    const PxTransform& bodyAToWorld, 
    const PxTransform& bodyBToWorld)
{
	Px1DConstraint* current = c;
	const AntiRollData* data = static_cast<const AntiRollData*>(constantBlock);
	PxVehicleDrive4W* v = data->mVehicle;
	const PxVehicleWheelsSimData& WheelsSimData = v->mWheelsSimData;
	const PxF32 wheelBase = (WheelsSimData.getWheelCentreOffset(PxVehicleDrive4WWheelOrder::eFRONT_LEFT) - WheelsSimData.getWheelCentreOffset(PxVehicleDrive4WWheelOrder::eREAR_LEFT)).magnitude();
	// turn left as positive position.
	const PxF32 steerRad = -data->steerRad;
	// motorcycle turn radius.
	const PxF32 r = (wheelBase / 2.0f) / (PxSin(steerRad / 2.0f));
	PxRigidDynamic* rigidDynamic = v->getRigidDynamicActor();
	PxScene* scene = rigidDynamic->getScene();
	// motorcycle velocity.
	const PxF32 u = rigidDynamic->getLinearVelocity().magnitude();
	// motorcycle acceleration around radius.
	const PxF32 a = u * u / r;
	const PxF32 g = scene->getGravity().magnitude();
	// target camber angle when motorcycle turn around radius.
	const PxF32 targetCamberRad = PxAtan2(a, g);

	const PxQuat qB = bodyBToWorld.q;

	PxQuat qA, twistUp;
	swing_twist_decomposition(qB, gUp, twistUp);
	PxQuat pitch(physx::shdfnd::degToRad(-30.0f), gRight);
	//PxQuat pitch(PxIdentity);
	PxQuat roll(targetCamberRad, gForward);
	qA = twistUp * roll * pitch;

	PxQuat qB2qA = qA.getConjugate() * qB;

	PxVec3 row[3];
	computeJacobianAxes(row, qA, qB);
	PxVec3 imp = qB2qA.getImaginaryPart();
	PxU32 rightDirection, upDirection;
	snippetvehicle::computeDirection(rightDirection, upDirection);
	PxU32 forwarDirection = (0 + 1 + 2) - rightDirection - upDirection;
	bool soft = false;
	angular(row[rightDirection], -imp[rightDirection], current++, soft);
	angular(row[forwarDirection], -imp[forwarDirection], current++, soft);
	angularVelocity(row[upDirection], -1, current++, soft);


    return current - c;
}

extern PxPhysics* gPhysics;

AntiRollContraints::ConstraintData AntiRollContraints::createConstraint(PxVehicleDrive4W* v)
{
	ConstraintData data;
	data.antiRoll.reset(new AntiRollContraints(v));
	data.constraint = gPhysics->createConstraint(NULL, v->getRigidDynamicActor(), *data.antiRoll.get(), AntiRollContraints::sShaders, sizeof(AntiRollData));
	return data;
}

PxConstraintShaderTable AntiRollContraints::sShaders = { 
	AntiRollContraints::antiRollConstraintSolverPrep, 
	NULL, 
	NULL, 
	PxConstraintFlag::Enum(0) 
};
