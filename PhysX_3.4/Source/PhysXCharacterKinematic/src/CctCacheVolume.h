#pragma once

#include "CctCharacterController.h"
namespace physx
{
	namespace Cct
	{
		class CharacterControllerManager;
		class CctCacheVolume : public Ps::UserAllocated
		{
		public:
			TriArray			mWorldTriangles;
#if PX_ENABLE_MTD_MOVEMENT
			MTDArray			mMTDs;
			PxExtendedVec3		mMTDCapsuleCenter;
#endif
			IntArray			mTriangleIndices;
			IntArray			mGeomStream;
			PxExtendedBounds3	mCacheBounds;
			PxU32				mCachedTriIndexIndex;
			mutable	PxU32		mCachedTriIndex[3];
			PxU32				mNbCachedStatic;
			PxU32				mNbCachedT;
			PxU32				mSQTimeStamp;
			CharacterControllerManager* mCctManager = nullptr;


		public:
			virtual ~CctCacheVolume();

			void				setCctManager(CharacterControllerManager* cm)
			{
				mCctManager = cm;
			}

			virtual PxExtendedBounds3 calcCacheBounds() const = 0;

			void onRelease(const PxBase& observed);

			void updateCachedShapesRegistration(PxU32 startIndex, bool unregister);

			bool sweep(const PxVec3& unitDir,
				const PxReal maxDist,
				const PxGeometry& geom0,
				const PxExtendedVec3& pos,
				const PxQuat& rot,
				PxSweepHit& sweepHit,
				PxHitFlags hitFlags = PxHitFlag::eDEFAULT,
				const PxReal inflation = 0.f);

		private:

			void updateTouchedGeomsCacheVolume(PxScene* scene,
				const PxExtendedBounds3& worldTemporalBox, PxQueryFilterData sceneQueryFilterData,
				PxQueryFilterCallback* filterCallback,
				const PxQuat& quatFromUp,
				bool bFirstUpdate
#if PX_ENABLE_MTD_MOVEMENT
				, const SweptVolume& swept_volume
#endif
			);
		};
	}
}