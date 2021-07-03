#pragma once
#include "PxConstraintDesc.h"
#include "extensions/PxConstraintExt.h"
#include "PxConstraint.h"
#include "vehicle/PxVehicleWheels.h"
#include "vehicle/PxVehicleDrive4W.h"
#include <memory>

using namespace physx;

class AntiRollContraints : public PxConstraintConnector
{
public:
	AntiRollContraints(PxVehicleDrive4W* vehicle)
	{
		mAntiRollData.mVehicle = vehicle;
	}
	virtual void* prepareData() override
	{
		return &mAntiRollData;
	}

	struct ConstraintData
	{
		std::unique_ptr<AntiRollContraints> antiRoll;
		PxConstraint* constraint;
	};

	static PxU32 antiRollConstraintSolverPrep(
		Px1DConstraint* constraints,
		PxVec3& body0WorldOffset,
		PxU32 maxConstraints,
		PxConstraintInvMassScale&,
		const void* constantBlock,
		const PxTransform& bodyAToWorld,
		const PxTransform& bodyBToWorld);

	static ConstraintData createConstraint(PxVehicleDrive4W* v);

	static PxConstraintShaderTable sShaders;

	virtual bool updatePvdProperties(physx::pvdsdk::PvdDataStream& pvdConnection,
		const PxConstraint* c,
		PxPvdUpdateType::Enum updateType) const override
	{
		PX_UNUSED(c); PX_UNUSED(updateType); PX_UNUSED(&pvdConnection); return true;
	}
	virtual void			onComShift(PxU32 actor) override { PX_UNUSED(actor); }

	virtual void			onOriginShift(const PxVec3& shift) override { PX_UNUSED(shift); }
	virtual void			onConstraintRelease() override
	{
		
	}
	virtual void* getExternalReference(PxU32& typeID) override
	{
		typeID = 100;
		return this;
	}

	virtual PxBase* getSerializable() override
	{
		return NULL;
	}
	virtual PxConstraintSolverPrep getPrep() const override
	{
		return NULL;
	}
	virtual const void* getConstantBlock() const override
	{
		return NULL;
	}
	struct AntiRollData
	{
		PxVehicleDrive4W* mVehicle;
		PxF32 steerRad;
	};
	AntiRollData mAntiRollData;
};

