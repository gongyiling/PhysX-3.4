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

	const PxQuat rotation = rigidDynamic->getGlobalPose().q;
	const PxVec3 left = rotation.rotate(-gRight);
	const PxF32 dot = left.dot(gUp);
	const PxF32 currentCamberRad = -PxAsin(dot);
	const PxVec3 Dir = rotation.rotate(gForward);
	const bool soft = false;
	if (soft)
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
    return 1;
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
