//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef SQ_AABBTREEQUERY_H
#define SQ_AABBTREEQUERY_H

#include "SqAABBTree.h"
#include "SqPruner.h"
#include "SqPrunerTestsSIMD.h"

#define SQ_AABB_DEBUG_BATCH_QUERY 1

#if SQ_AABB_DEBUG_BATCH_QUERY
#include <iostream>

union Vec3U
{
	physx::Vec3V v;		// SSE 4 x float vector
	float a[4];		// scalar array of 4 floats
};

PX_FORCE_INLINE std::ostream& operator <<(std::ostream& os, const physx::Vec3V& V)
{
	Vec3U U;
	U.v = V;
	os << U.a[0] << ',' << U.a[1] << ',' << U.a[2];
	return os;
}

PX_FORCE_INLINE void printMinMax(const physx::Vec3V& minV, const physx::Vec3V& maxV, bool bResult)
{
	// std::cerr << minV << '\t' << maxV << '\t' << bResult << std::endl;
}

PX_FORCE_INLINE void printCenterExtend2(const physx::Vec3V& c2, const physx::Vec3V& e2, bool bResult)
{
	const float half = 0.5f;
	const physx::FloatV halfV = physx::FLoad(half);

	const physx::Vec4V extents_ = physx::V4Scale(e2, halfV);
	const physx::Vec4V center_ = physx::V4Scale(c2, halfV);
	printMinMax(physx::V4Sub(center_, extents_), physx::V4Add(center_, extents_), bResult);
}

#else

PX_FORCE_INLINE void printMinMax(const physx::Vec3V& minV, const physx::Vec3V& maxV, bool bResult)
{
	
}

PX_FORCE_INLINE void printCenterExtend2(const physx::Vec3V& c2, const physx::Vec3V& e2, bool bResult)
{
	
}

#endif

namespace physx
{
	namespace Sq
	{
		#define RAW_TRAVERSAL_STACK_SIZE 256

		//////////////////////////////////////////////////////////////////////////

		static PX_FORCE_INLINE void getBoundsTimesTwo(Vec4V& center, Vec4V& extents, const PxBounds3* boxes, PoolIndex poolIndex)
		{
			const PxBounds3* objectBounds = boxes + poolIndex;

			const Vec4V minV = V4LoadU(&objectBounds->minimum.x);
			const Vec4V maxV = V4LoadU(&objectBounds->maximum.x);

			center = V4Add(maxV, minV);
			extents = V4Sub(maxV, minV);
		}

		//////////////////////////////////////////////////////////////////////////

		template<typename Test, typename Tree, typename Node>
		class AABBTreeOverlap
		{
		public:
			bool operator()(const PrunerPayload* objects, const PxBounds3* boxes, const Tree& tree, const Test& test, PrunerCallback& visitor)
			{
				using namespace Cm;
				Ps::InlineArray<const Node*, RAW_TRAVERSAL_STACK_SIZE> stack;
				stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
				const Node* const nodeBase = tree.getNodes();
				stack[0] = nodeBase;
				PxU32 stackIndex = 1;

				while (stackIndex > 0)
				{
					const Node* node = stack[--stackIndex];
					Vec3V center, extents;
					node->getAABBCenterExtentsV(&center, &extents);
					while (test(center, extents))
					{
						if (node->isLeaf())
						{
							PxU32 nbPrims = node->getNbPrimitives();
							const bool doBoxTest = nbPrims > 1;
							const PxU32* prims = node->getPrimitives(tree.getIndices());
							while (nbPrims--)
							{
								const PxU32* prunableIndex = prims;
								prims++;

								const PoolIndex poolIndex = *prunableIndex;
								if (doBoxTest)
								{
									Vec4V center2, extents2;
									getBoundsTimesTwo(center2, extents2, boxes, poolIndex);

									const float half = 0.5f;
									const FloatV halfV = FLoad(half);

									const Vec4V extents_ = V4Scale(extents2, halfV);
									const Vec4V center_ = V4Scale(center2, halfV);

									if (!test(Vec3V_From_Vec4V(center_), Vec3V_From_Vec4V(extents_)))
										continue;
								}

								PxReal unusedDistance;
								if (!visitor.invoke(unusedDistance, objects[poolIndex]))
									return false;
							}
							break;
						}

						const Node* children = node->getPos(nodeBase);

						node = children;
						stack[stackIndex++] = children + 1;
						if(stackIndex == stack.capacity())
							stack.resizeUninitialized(stack.capacity() * 2);
						node->getAABBCenterExtentsV(&center, &extents);
					}
				}
				return true;
			}
		};

		//////////////////////////////////////////////////////////////////////////

		template <bool tInflate, typename Tree, typename Node> // use inflate=true for sweeps, inflate=false for raycasts
		static PX_FORCE_INLINE bool doLeafTest(const Node* node, Gu::RayAABBTest& test, PxReal& md, PxReal oldMaxDist,
			const PrunerPayload* objects, const PxBounds3* boxes, const Tree& tree,
			PxReal& maxDist, PrunerCallback& pcb)
		{
			PxU32 nbPrims = node->getNbPrimitives();
			const bool doBoxTest = nbPrims > 1;
			const PxU32* prims = node->getPrimitives(tree.getIndices());
			while (nbPrims--)
			{
				const PxU32* prunableIndex = prims;
				prims++;

				const PoolIndex poolIndex = *prunableIndex;
				if (doBoxTest)
				{
					Vec4V center_, extents_;
					getBoundsTimesTwo(center_, extents_, boxes, poolIndex);
					const PxU32 b = test.check<tInflate>(Vec3V_From_Vec4V(center_), Vec3V_From_Vec4V(extents_));
					printCenterExtend2(center_, extents_, b);
					if (!b)
						continue;
				}

				if (!pcb.invoke(md, objects[poolIndex]))
					return false;

				if (md < oldMaxDist)
				{
					maxDist = md;
					test.setDistance(md);
				}
			}
			return true;
		}

		//////////////////////////////////////////////////////////////////////////

		template <bool tInflate, typename Tree, typename Node> // use inflate=true for sweeps, inflate=false for raycasts
		class AABBTreeRaycast
		{
		public:
			bool operator()(
				const PrunerPayload* objects, const PxBounds3* boxes, const Tree& tree,
				const PxVec3& origin, const PxVec3& unitDir, PxReal& maxDist, const PxVec3& inflation,
				PrunerCallback& pcb)
			{
				using namespace Cm;

				// PT: we will pass center*2 and extents*2 to the ray-box code, to save some work per-box
				// So we initialize the test with values multiplied by 2 as well, to get correct results
				Gu::RayAABBTest test(origin*2.0f, unitDir*2.0f, maxDist, inflation*2.0f);

				Ps::InlineArray<const Node*, RAW_TRAVERSAL_STACK_SIZE> stack;
				stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
				const Node* const nodeBase = tree.getNodes();
				stack[0] = nodeBase;
				PxU32 stackIndex = 1;

				PxReal oldMaxDist;
				while (stackIndex--)
				{
					const Node* node = stack[stackIndex];
					Vec3V center, extents;
					node->getAABBCenterExtentsV2(&center, &extents);
					const PxU32 b = test.check<tInflate>(center, extents);
					printCenterExtend2(center, extents, b);
					if (b)	// TODO: try timestamp ray shortening to skip this
					{
						PxReal md = maxDist; // has to be before the goto below to avoid compile error
						while (!node->isLeaf())
						{
							const Node* children = node->getPos(nodeBase);

							Vec3V c0, e0;
							children[0].getAABBCenterExtentsV2(&c0, &e0);
							const PxU32 b0 = test.check<tInflate>(c0, e0);
							printCenterExtend2(c0, e0, b0);

							Vec3V c1, e1;
							children[1].getAABBCenterExtentsV2(&c1, &e1);
							const PxU32 b1 = test.check<tInflate>(c1, e1);
							printCenterExtend2(c1, e1, b1);

							if (b0 && b1)	// if both intersect, push the one with the further center on the stack for later
							{
#if SQ_AABB_DEBUG_BATCH_QUERY
								const PxU32 bit = 1;
#else
								// & 1 because FAllGrtr behavior differs across platforms
								const PxU32 bit = FAllGrtr(V3Dot(V3Sub(c1, c0), test.mDir), FZero()) & 1;
#endif
								stack[stackIndex++] = children + bit;
								node = children + (1 - bit);
								if (stackIndex == stack.capacity())
									stack.resizeUninitialized(stack.capacity() * 2);
							}
							else if (b0)
								node = children;
							else if (b1)
								node = children + 1;
							else
								goto skip_leaf_code;
						}

						oldMaxDist = maxDist; // we copy since maxDist can be updated in the callback and md<maxDist test below can fail

						if (!doLeafTest<tInflate, Tree, Node>(node, test, md, oldMaxDist,
							objects, boxes, tree,
							maxDist,
							pcb))
							return false;
					skip_leaf_code:;
					}
				}
				return true;
			}
		};


		//////////////////////////////////////////////////////////////////////////

		struct BatchRay
		{
			BatchRay(SqBatchRay& inRays)
				:sqBatchRay(inRays)
			{
				
			}

			PX_FORCE_INLINE SqRayMask check(const Vec3V minV, const Vec3V maxV)
			{
				SqRayMask thisMask = 0;
				const uint32_t rayNum = sqBatchRay.rays.size();
				for (uint32_t i = 0; i < rayNum; i++)
				{
					if (masked(i))
					{
						continue;
					}
					SqRay& ray = sqBatchRay.rays[i];
					const PxU32 intersect = BAllEqTTTT(BAnd(V3IsGrtrOrEq(ray.maxV, minV), V3IsGrtrOrEq(maxV, ray.minV)));
					thisMask |= ((1 - intersect) << i);
					printMinMax(minV, maxV, intersect);
				}
				sqBatchRay.mask |= thisMask;
				return thisMask;
			}

			PX_FORCE_INLINE bool masked(uint32_t i) const
			{
				return sqBatchRay.isMasked(i);
			}

			PX_FORCE_INLINE void restore(SqRayMask thisMask) const
			{
				sqBatchRay.mask &= ~thisMask;
			}

			PX_FORCE_INLINE bool isEmpty() const
			{
				return sqBatchRay.isEmpty();
			}

			void cacheMaxDist()
			{
				const uint32_t rayNum = sqBatchRay.rays.size();
				for (uint32_t i = 0; i < rayNum; i++)
				{
					SqRay& ray = sqBatchRay.rays[i];
					ray.oldMaxDist = ray.md = ray.maxDist;
				}
			}
			SqBatchRay& sqBatchRay;
		};

		template <SqRayDirection Direction, typename Tree, typename Node> // use inflate=true for sweeps, inflate=false for raycasts
		class AABBTreeBatchRaycast
		{
		public:

			struct BatchRaycastSharedParams
			{
				BatchRaycastSharedParams(const PrunerPayload* inObjects, const PxBounds3* inBoxes, const Tree& inTree, BatchRay& inBatchRay)
				: objects(inObjects)
				, boxes(inBoxes)
				, tree(inTree)
				, batchRay(inBatchRay)
				{
					
				}

				const PrunerPayload* objects;
				const PxBounds3* boxes;
				const Tree& tree;
				BatchRay& batchRay;
			};

			static PX_FORCE_INLINE void doLeafTest(const BatchRaycastSharedParams& sharedParams, const Node* node)
			{
				PxU32 nbPrims = node->getNbPrimitives();
				const bool doBoxTest = nbPrims > 1;
				const PxU32* prims = node->getPrimitives(sharedParams.tree.getIndices());
				BatchRay& test = sharedParams.batchRay;
				while (nbPrims--)
				{
					const PxU32* prunableIndex = prims;
					prims++;

					const PoolIndex poolIndex = *prunableIndex;
					SqRayMask mask = 0;
					if (doBoxTest)
					{
						const PxBounds3* objectBounds = sharedParams.boxes + poolIndex;
						const Vec3V minV = V3LoadU(&objectBounds->minimum.x);
						const Vec3V maxV = V3LoadU(&objectBounds->maximum.x);
						mask = test.check(minV, maxV);
					}

					const uint32_t rayNum = test.sqBatchRay.rays.size();
					for (uint32_t i = 0; i < rayNum; i++)
					{
						if (test.masked(i))
						{
							continue;
						}
						SqRay& ray = test.sqBatchRay.rays[i];
						if (!ray.pcb->invoke(ray.md, sharedParams.objects[poolIndex]))
						{
							test.sqBatchRay.mask |= (1 << i);
						}
						else if (ray.md < ray.oldMaxDist)
						{
							ray.setDistance(ray.md, Direction);
						}
					}

					test.restore(mask);

					if (test.isEmpty())
					{
						break;
					}
				}
			}

			static void getAABBMinMaxV(Vec3V& minV, Vec3V& maxV, const Node* node)
			{
				Vec4V minV4, maxV4;
				node->getAABBMinMaxV(&minV4, &maxV4);
				minV = Vec3V_From_Vec4V(minV4);
				maxV = Vec3V_From_Vec4V(maxV4);
			}

			void doBatchRaycast(const BatchRaycastSharedParams& sharedParams, const Vec3V& minV, const Vec3V& maxV, const Node* node)
			{
				const SqRayMask mask = sharedParams.batchRay.check(minV, maxV);
				if (!sharedParams.batchRay.isEmpty())
				{
					if(!node->isLeaf())
					{
						const Node* const nodeBase = sharedParams.tree.getNodes();
						const Node* children = node->getPos(nodeBase);

						Vec3V minCV[2], maxCV[2];
						getAABBMinMaxV(minCV[0], maxCV[0], &children[0]);

						getAABBMinMaxV(minCV[1], maxCV[1], &children[1]);
#if SQ_AABB_DEBUG_BATCH_QUERY
						PxU32 bit1 = 1;
#else
						PxU32 bit1 = 0;
						const Vec3V S = V3Sub(minCV[1], minCV[0]);
						switch (Direction)
						{
						case SqRayDirection::SRD_PosX:
							bit1 = FAllGrtr(V3GetX(S), FZero());
							break;
						case SqRayDirection::SRD_NegX:
							bit1 = FAllGrtr(FZero(), V3GetX(S));
							break;
						case SqRayDirection::SRD_PosY:
							bit1 = FAllGrtr(V3GetY(S), FZero());
							break;
						case SqRayDirection::SRD_NegY:
							bit1 = FAllGrtr(FZero(), V3GetY(S));
							break;
						case SqRayDirection::SRD_PosZ:
							bit1 = FAllGrtr(V3GetZ(S), FZero());
							break;
						case SqRayDirection::SRD_NegZ:
							bit1 = FAllGrtr(FZero(), V3GetZ(S));
							break;
						}
#endif
						const PxU32 bit0 = 1 - bit1;
						// if child1 is far from child0 in the direction of ray, go child0 first.
						doBatchRaycast(sharedParams, minCV[bit0], maxCV[bit0], &children[bit0]);
						doBatchRaycast(sharedParams, minCV[bit1], maxCV[bit1], &children[bit1]);
					}
					else
					{
						sharedParams.batchRay.cacheMaxDist();
						doLeafTest(sharedParams, node);
					}
				}
				sharedParams.batchRay.restore(mask);
			}

			void operator()(
				const PrunerPayload* objects, const PxBounds3* boxes, const Tree& tree,
				SqBatchRay& sqBatchRay)
			{
				using namespace Cm;

				BatchRay batchRay(sqBatchRay);
				const Node* const nodeBase = tree.getNodes();
				BatchRaycastSharedParams sharedParams(objects, boxes, tree, batchRay);

				Vec3V minV, maxV;
				getAABBMinMaxV(minV, maxV, nodeBase);
				doBatchRaycast(sharedParams, minV, maxV, nodeBase);
			}
		};
	}
}

#endif   // SQ_AABBTREEQUERY_H
