#include "AntiRollContraints.h"

PxU32 AntiRollContraints::antiRollConstraintSolverPrep(Px1DConstraint* constraints, 
    PxVec3& body0WorldOffset, 
    PxU32 maxConstraints,
    PxConstraintInvMassScale&, 
    const void* constantBlock, 
    const PxTransform& bodyAToWorld, 
    const PxTransform& bodyBToWorld)
{
    return PxU32();
}
