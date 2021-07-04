#include "AntiRollContraints.h"
#include "PsMathUtils.h"
#include "foundation/PxMath.h"
#include "vehicle/PxVehicleDrive4W.h"
#include "PxScene.h"
#include "PxPhysics.h"

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
	PxQuat& swing,
	PxQuat& twist)
{
	PxVec3 ra(rotation.x, rotation.y, rotation.z); // rotation axis
	PxVec3 p = projection(ra, direction); // return projection v1 on to v2  (parallel component)
	twist = PxQuat(p.x, p.y, p.z, rotation.w);
	twist.normalize();
	swing = rotation * twist.getConjugate();
}

PxU32 AntiRollContraints::antiRollConstraintSolverPrep(Px1DConstraint* c, 
    PxVec3& body0WorldOffset, 
    PxU32 maxConstraints,
    PxConstraintInvMassScale&, 
    const void* constantBlock, 
    const PxTransform& bodyAToWorld, 
    const PxTransform& bodyBToWorld)
{
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

	const PxQuat rotation = bodyBToWorld.q;

	const PxVec3 left = rotation.rotate(-gRight);
	const PxF32 dot = left.dot(gUp);
	const PxF32 currentCamberRad = -PxAsin(dot);

	// swing = twistUp * twistForward
	// rotation = twistUp * twistForward * twistRight

	PxQuat swing, twistRight;
	swing_twist_decomposition(rotation, gRight, swing, twistRight);
	PxQuat twistUp, twistForward;
	swing_twist_decomposition(swing, gForward, twistUp, twistForward);

	const PxVec3 Dir = twistUp.rotate(gForward);
	const bool soft = false;
	if (true)
	{
		c->solveHint = PxU16(PxConstraintSolveHint::eNONE);
		c->linear0 = PxVec3(0.f);		c->angular0 = Dir;
		c->linear1 = PxVec3(0.f);		c->angular1 = Dir;
		c->mods.spring.stiffness = 100;
		c->mods.spring.damping = 10;
		c->flags = Px1DConstraintFlag::eSPRING | Px1DConstraintFlag::eACCELERATION_SPRING;
	}
	else
	{
		c->solveHint = PxU16(PxConstraintSolveHint::eEQUALITY);
		c->linear0 = PxVec3(0.f);		c->angular0 = Dir;
		c->linear1 = PxVec3(0.f);		c->angular1 = Dir;
		c->flags = 0;
	}
	c->geometricError = targetCamberRad - currentCamberRad;
	c++;
	PxVec3 right = swing.rotate(gRight);
	const PxF32 twistAngle = twistRight.getAngle();
	if (true)
	{
		c->solveHint = PxU16(PxConstraintSolveHint::eNONE);
		c->linear0 = PxVec3(0.f);		c->angular0 = right;
		c->linear1 = PxVec3(0.f);		c->angular1 = right;
		c->mods.spring.stiffness = 100;
		c->mods.spring.damping = 10;
		c->flags = Px1DConstraintFlag::eSPRING | Px1DConstraintFlag::eACCELERATION_SPRING;
	}
	else
	{
		c->solveHint = PxU16(PxConstraintSolveHint::eEQUALITY);
		c->linear0 = PxVec3(0.f);		c->angular0 = right;
		c->linear1 = PxVec3(0.f);		c->angular1 = right;
		c->flags = 0;
	}
	c->geometricError = physx::shdfnd::degToRad(30.0f) - twistAngle;
    return 2;
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
