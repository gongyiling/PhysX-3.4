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

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************

#include <ctype.h>

#include "PxPhysicsAPI.h"

#include "../SnippetCommon/SnippetPrint.h"
#include "../SnippetCommon/SnippetPVD.h"
#include "../SnippetUtils/SnippetUtils.h"
#include <iostream>
#include <array>
#include <PsArray.h>
#include <vector>

using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PxFoundation*			gFoundation = NULL;
PxPhysics*				gPhysics	= NULL;

PxDefaultCpuDispatcher*	gDispatcher = NULL;
PxScene*				gScene		= NULL;

PxMaterial*				gMaterial	= NULL;

PxPvd*                  gPvd        = NULL;

PxReal stackZ = 10.0f;

PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
	PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

void createStack(const PxTransform& t, PxU32 size, PxReal halfExtent)
{
	PxShape* shape = gPhysics->createShape(PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for(PxU32 i=0; i<size;i++)
	{
		for(PxU32 j=0;j<size-i;j++)
		{
			PxTransform localTm(PxVec3(PxReal(j*2) - PxReal(size-i), PxReal(i*2+1), 0) * halfExtent);
			PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

struct RaycastCallback : PxRaycastCallback 
{
	using PxRaycastCallback::PxRaycastCallback;
	virtual PxAgain processTouches(const PxRaycastHit* buffer, PxU32 nbHits) override
	{
		for (PxU32 i = 0; i < nbHits; i++)
		{
			hits.pushBack(buffer[i]);
		}
		return true;
	}
	shdfnd::Array<PxRaycastHit> hits;
	PX_FORCE_INLINE bool hasAnyHitsEx() { return PxRaycastCallback::hasAnyHits() || !hits.empty(); }
};

PxRigidDynamic* PxCreateDynamic(PxPhysics& sdk,
	const PxTransform& transform,
	PxShape& shape,
	PxReal density)
{
	PxRigidDynamic* actor = sdk.createRigidDynamic(transform);
	if (actor)
	{
		actor->attachShape(shape);
		PxRigidBodyExt::updateMassAndInertia(*actor, density);
	}
	return actor;
}

static void checkHit(const PxRaycastHit& a, const PxRaycastHit& b)
{
	PX_ASSERT(a.distance == b.distance);
	PX_ASSERT(a.normal == b.normal);
	PX_ASSERT(a.position == b.position);

	PX_ASSERT(a.actor == b.actor);
	PX_ASSERT(a.shape == b.shape);

	PX_ASSERT(a.flags == b.flags);
}

static void checkCallback(const RaycastCallback& a, const RaycastCallback& b)
{
	PX_ASSERT(a.hasBlock == b.hasBlock)
	if (a.hasBlock)
	{
		checkHit(a.block, b.block);
	}
	PX_ASSERT(a.hits.size() == b.hits.size());
	for (PxU32 i = 0; i < a.hits.size(); i++)
	{
		checkHit(a.hits[i], b.hits[i]);
	}
}

struct RaycastHitArray
{
	PxRaycastHit hits[128];
};

static void testBatchQuery(const PxVec3* origin, PxU32 numOrigin, const PxVec3& dir, PxReal dist, bool block, bool anyHit)
{
	std::vector<RaycastHitArray> hits;
	hits.resize(numOrigin);

	shdfnd::Array<RaycastCallback> callbacks;
	callbacks.resizeUninitialized(numOrigin);

	shdfnd::Array<PxRay> rays;
	rays.resizeUninitialized(numOrigin);

	for (PxU32 i = 0; i < numOrigin; i++)
	{
		if (block)
		{
			RaycastCallback* callback = new (&callbacks[i]) RaycastCallback(nullptr, 0);
			new (&rays[i]) PxRay(callback, origin[i], dist);
		}
		else
		{
			RaycastCallback* callback = new (&callbacks[i]) RaycastCallback(hits[i].hits, 128);
			new (&rays[i]) PxRay(callback, origin[i], dist);
		}
	}
	PxQueryFilterData queryFilterData;
	if (anyHit)
	{
		queryFilterData.flags |= PxQueryFlag::eANY_HIT;
	}
	gScene->batchRaycast(rays.begin(), rays.end(), dir, PxHitFlags(PxHitFlag::eDEFAULT), queryFilterData);

	for (PxU32 i = 0; i < numOrigin; i++)
	{
		RaycastHitArray hit;
		RaycastCallback callback((block ? nullptr : hit.hits), (block ? 0 : 128));
		gScene->raycast(origin[i], dir, dist, callback, PxHitFlags(PxHitFlag::eDEFAULT), queryFilterData);
		checkCallback(callback, callbacks[i]);
	}
}

static void testBatchQuery(const PxVec3* origin, PxU32 numOrigin, const PxVec3& dir, PxReal dist)
{
	testBatchQuery(origin, numOrigin, dir, dist, true, true);
	testBatchQuery(origin, numOrigin, dir, dist, true, false);
	testBatchQuery(origin, numOrigin, dir, dist, false, true);
	testBatchQuery(origin, numOrigin, dir, dist, false, false);
}

void initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_FOUNDATION_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
	gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(),true,gPvd);

	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher	= gDispatcher;
	sceneDesc.filterShader	= PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if(pvdClient)
	{
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	PxRigidStatic * groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	PxTransform Transform = groundPlane->getGlobalPose();

	//groundPlane->setGlobalPose(PxTransform(PxVec3(0, 1, 0)));

	for(PxU32 i=0;i<5;i++)
		createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);

	if(!interactive)
		createDynamic(PxTransform(PxVec3(0,40,100)), PxSphereGeometry(10), PxVec3(0,-50,-100));
	
	std::vector<PxVec3> origins;

	for (PxI32 x = -300; x < 300; x += 10)
	{
		for (PxI32 y = -300; y < 300; y += 10)
		{
			for (PxI32 z = -300; z < 300; z += 10)
			{
				origins.emplace_back(PxVec3(x, y, z));
			}
		}
	}
	std::vector<PxReal> distances = {10, 100, 1000};

	std::vector<PxVec3> dirs = {
		PxVec3(0, 0, 1)
		, PxVec3(0, 0, -1)
		, PxVec3(0, 1, 0)
		, PxVec3(0, -1, 0)
		, PxVec3(1, 0, 0)
		, PxVec3(-1, 0, 0)
	};
	
	for (PxU32 i = 0; i < distances.size(); i++)
	{
		for (PxU32 j = 0; j < origins.size(); j++)
		{
			testBatchQuery(origins.data(), origins.size(), dirs[j], distances[i]);
		}
	}

	Transform.p = PxVec3(0, -50, 0);
	groundPlane->setGlobalPose(Transform);

}

void stepPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->simulate(1.0f/60.0f);
	gScene->fetchResults(true);
}
	
void cleanupPhysics(bool interactive)
{
	PX_UNUSED(interactive);
	gScene->release();
	gDispatcher->release();
	gPhysics->release();	
	PxPvdTransport* transport = gPvd->getTransport();
	gPvd->release();
	transport->release();
	
	gFoundation->release();
	
	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform& camera)
{
	switch(toupper(key))
	{
	case 'B':	createStack(PxTransform(PxVec3(0,0,stackZ-=10.0f)), 10, 2.0f);						break;
	case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0,0,-1))*200);	break;
	}
}

int snippetMain(int, const char*const*)
{
#ifdef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for(PxU32 i=0; i<frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}
