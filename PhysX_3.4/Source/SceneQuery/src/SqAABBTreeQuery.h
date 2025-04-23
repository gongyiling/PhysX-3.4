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

					if (!test.check<tInflate>(Vec3V_From_Vec4V(center_), Vec3V_From_Vec4V(extents_)))
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
					if (test.check<tInflate>(center, extents))	// TODO: try timestamp ray shortening to skip this
					{
						PxReal md = maxDist; // has to be before the goto below to avoid compile error
						while (!node->isLeaf())
						{
							const Node* children = node->getPos(nodeBase);

							Vec3V c0, e0;
							children[0].getAABBCenterExtentsV2(&c0, &e0);
							const PxU32 b0 = test.check<tInflate>(c0, e0);

							Vec3V c1, e1;
							children[1].getAABBCenterExtentsV2(&c1, &e1);
							const PxU32 b1 = test.check<tInflate>(c1, e1);

							if (b0 && b1)	// if both intersect, push the one with the further center on the stack for later
							{
								// & 1 because FAllGrtr behavior differs across platforms
								const PxU32 bit = FAllGrtr(V3Dot(V3Sub(c1, c0), test.mDir), FZero()) & 1;
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

		union Vec3U
		{
			Vec3V v;		// SSE 4 x float vector
			float a[4];		// scalar array of 4 floats
		};

		struct BatchRay
		{
			SqRayPtrArray rays;

			PX_FORCE_INLINE static bool intersect(PxReal x1Min, PxReal x1Max, PxReal x2Min, PxReal x2Max)
			{
				return !(x1Max < x2Min || x2Max < x1Min);
			}

			PX_FORCE_INLINE static bool intersect(PxReal x1, PxReal x2Min, PxReal x2Max)
			{
				return x1 >= x2Min && x1 <= x2Max;
			}

			template<bool TInflate, SqRayDirection Direction>
			PX_FORCE_INLINE SqRayPtrArray check(const Vec3V minV, const Vec3V maxV)
			{
				SqRayPtrArray filteredRays;
				for (uint32_t i = 0; i < rays.size();)
				{
					SqRay& ray = *rays[i];
					PX_ASSERT(!ray.canExit);

					Vec3U eMinV;
					eMinV.v = TInflate ? V3Sub(minV, V3LoadU(ray.inflation)) : minV;
					Vec3U eMaxV;
					eMaxV.v = TInflate ? V3Add(maxV, V3LoadU(ray.inflation)) : maxV;

					const PxVec3& o = ray.pxRay->origin;
					PxReal minBox, maxBox, minRay, maxRay;
					PxReal secondAxis, minSecondAxis, maxSecondAxis;
					PxReal thirdAxis, minThirdAxis, maxThirdAxis;

					if (Direction == SRD_PosX || Direction == SRD_NegX)
					{
						minBox = eMinV.a[0];
						maxBox = eMaxV.a[0];
						if (Direction == SRD_PosX)
						{
							minRay = o.x;
							maxRay = o.x + ray.maxDist;
						}
						else
						{
							minRay = o.x - ray.maxDist;
							maxRay = o.x;
						}
						secondAxis = o.y;
						minSecondAxis = eMinV.a[1];
						maxSecondAxis = eMaxV.a[1];

						thirdAxis = o.z;
						minThirdAxis = eMinV.a[2];
						maxThirdAxis = eMaxV.a[2];
					}
					else if (Direction == SRD_PosY || Direction == SRD_NegY)
					{
						minBox = eMinV.a[1];
						maxBox = eMaxV.a[1];
						if (Direction == SRD_PosY)
						{
							minRay = o.y;
							maxRay = o.y + ray.maxDist;
						}
						else
						{
							minRay = o.y - ray.maxDist;
							maxRay = o.y;
						}
						secondAxis = o.x;
						minSecondAxis = eMinV.a[0];
						maxSecondAxis = eMaxV.a[0];

						thirdAxis = o.z;
						minThirdAxis = eMinV.a[2];
						maxThirdAxis = eMaxV.a[2];
					}
					else
					{
						minBox = eMinV.a[2];
						maxBox = eMaxV.a[2];
						if (Direction == SRD_PosZ)
						{
							minRay = o.z;
							maxRay = o.z + ray.maxDist;
						}
						else
						{
							minRay = o.z - ray.maxDist;
							maxRay = o.z;
						}
						secondAxis = o.x;
						minSecondAxis = eMinV.a[0];
						maxSecondAxis = eMaxV.a[0];

						thirdAxis = o.y;
						minThirdAxis = eMinV.a[1];
						maxThirdAxis = eMaxV.a[1];
					}

					const bool isIntersect = intersect(minBox, maxBox, minRay, maxRay)
						&& intersect(secondAxis, minSecondAxis, maxSecondAxis)
						&& intersect(thirdAxis, minThirdAxis, maxThirdAxis);

					if (!isIntersect)
					{
						filteredRays.pushBack(rays[i]);
						rays.removeAtSwap(i);
					}
					else
					{
						i++;
					}
				}
				return filteredRays;
			}

			void restore(const SqRayPtrArray& inRays)
			{
				if (inRays.empty())
				{
					return;
				}

				const uint32_t oldSize = rays.size();
				rays.resize(rays.size() + inRays.size());
				for (uint32_t i = 0; i < inRays.size(); i++)
				{
					rays[i + oldSize] = inRays[i];
				}
			}

			bool filter()
			{
				SqRayPtrArray newRays;
				for (uint32_t i = 0; i < rays.size(); i++)
				{
					if (!rays[i]->canExit)
					{
						newRays.pushBack(rays[i]);
					}
				}
				rays = newRays;
				return hasRay();
			}

			bool hasRay() const
			{
				return !rays.empty();
			}

			void cacheMaxDist()
			{
				for (uint32_t i = 0; i < rays.size(); i++)
				{
					SqRay* ray = rays[i];
					ray->oldMaxDist = ray->md = ray->maxDist;
				}
			}

			void setCanExit(SqRay* ray)
			{
				ray->canExit = true;
			}
		};

		template <bool tInflate, SqRayDirection Direction, typename Tree, typename Node> // use inflate=true for sweeps, inflate=false for raycasts
		class AABBTreeBatchRaycast
		{
		public:

			struct BatchRaycastSharedParams
			{
				BatchRaycastSharedParams(const PrunerPayload* inObjects, const PxBounds3* inBoxes, const Tree& inTree)
				: objects(inObjects)
				, boxes(inBoxes)
				, tree(inTree)
				{
					
				}

				const PrunerPayload* objects;
				const PxBounds3* boxes;
				const Tree& tree;
			};

			static PX_FORCE_INLINE void doLeafTest(const BatchRaycastSharedParams& sharedParams, BatchRay& test, const Node* node)
			{
				PxU32 nbPrims = node->getNbPrimitives();
				const bool doBoxTest = nbPrims > 1;
				const PxU32* prims = node->getPrimitives(sharedParams.tree.getIndices());
				SqRayPtrArray filteredRays;
				while (nbPrims--)
				{
					const PxU32* prunableIndex = prims;
					prims++;

					const PoolIndex poolIndex = *prunableIndex;
					if (doBoxTest)
					{
						const PxBounds3* objectBounds = sharedParams.boxes + poolIndex;
						const Vec4V minV = V4LoadU(&objectBounds->minimum.x);
						const Vec4V maxV = V4LoadU(&objectBounds->maximum.x);

						filteredRays = test.check<tInflate, Direction>(minV, maxV);
					}

					bool hasExit = false;
					for (uint32_t i = 0; i < test.rays.size(); i++)
					{
						SqRay* ray = test.rays[i];
						if (!ray->pcb->invoke(ray->md, sharedParams.objects[poolIndex]))
						{
							test.setCanExit(ray);
							hasExit = true;
						}
						else if (ray->md < ray->oldMaxDist)
						{
							ray->maxDist = ray->md;
						}
					}

					test.restore(filteredRays);

					if (hasExit)
					{
						if (!test.filter())
						{
							break;
						}
					}
				}
			}

			void doBatchRaycast(const BatchRaycastSharedParams& sharedParams, BatchRay& batchRay, const Node* node)
			{
				Vec3V minV, maxV;
				node->getAABBMinMaxV(&minV, &maxV);
				const SqRayPtrArray filteredRays = batchRay.check<tInflate, Direction>(minV, minV);
				if (batchRay.hasRay())
				{
					if(!node->isLeaf())
					{
						const Node* const nodeBase = sharedParams.tree.getNodes();
						const Node* children = node->getPos(nodeBase);
						doBatchRaycast(sharedParams, batchRay, &children[0]);
						doBatchRaycast(sharedParams, batchRay, &children[1]);
					}
					else
					{
						batchRay.cacheMaxDist();
						doLeafTest(sharedParams, batchRay, node);
					}
				}
				batchRay.restore(filteredRays);
			}

			SqRayPtrArray operator()(
				const PrunerPayload* objects, const PxBounds3* boxes, const Tree& tree,
				const SqRayPtrArray& rays)
			{
				using namespace Cm;
				BatchRay batchRay;
				batchRay.rays = rays;
				const Node* const nodeBase = tree.getNodes();
				BatchRaycastSharedParams sharedParams(objects, boxes, tree);
				doBatchRaycast(sharedParams, batchRay, nodeBase);
				return batchRay.rays;
			}
		};
	}
}

#endif   // SQ_AABBTREEQUERY_H
