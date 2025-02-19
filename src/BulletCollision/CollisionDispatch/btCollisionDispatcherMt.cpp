/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btCollisionDispatcherMt.h"
#include "LinearMath/btQuickprof.h"

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "LinearMath/btPoolAllocator.h"
#include "BulletCollision/CollisionDispatch/btCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include <BulletCable/btCable.h>

btCollisionDispatcherMt::btCollisionDispatcherMt(btCollisionConfiguration* config, int grainSize)
	: btCollisionDispatcher(config)
{
	m_batchManifoldsPtr.resize(BT_MAX_THREAD_COUNT);
	m_batchReleasePtr.resize(BT_MAX_THREAD_COUNT);

	m_batchUpdating = false;
	m_grainSize = grainSize;  // iterations per task
}

btPersistentManifold* btCollisionDispatcherMt::getNewManifold(const btCollisionObject* body0, const btCollisionObject* body1)
{
	//optional relative contact breaking threshold, turned on by default (use setDispatcherFlags to switch off feature for improved performance)

	btScalar contactBreakingThreshold = (m_dispatcherFlags & btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD) ? btMin(body0->getCollisionShape()->getContactBreakingThreshold(gContactBreakingThreshold), body1->getCollisionShape()->getContactBreakingThreshold(gContactBreakingThreshold))
																																: gContactBreakingThreshold;

	btScalar contactProcessingThreshold = btMin(body0->getContactProcessingThreshold(), body1->getContactProcessingThreshold());

	void* mem = m_persistentManifoldPoolAllocator->allocate(sizeof(btPersistentManifold));
	if (NULL == mem)
	{
		//we got a pool memory overflow, by default we fallback to dynamically allocate memory. If we require a contiguous contact pool then assert.
		if ((m_dispatcherFlags & CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION) == 0)
		{
			mem = btAlignedAlloc(sizeof(btPersistentManifold), 16);
		}
		else
		{
			btAssert(0);
			//make sure to increase the m_defaultMaxPersistentManifoldPoolSize in the btDefaultCollisionConstructionInfo/btDefaultCollisionConfiguration
			return 0;
		}
	}
	// RigidBody only
	btPersistentManifold* manifold;
	if (body0->getInternalType() == 2 && body1->getInternalType() == 2) 
 		manifold = new (mem) btPersistentManifold(body0, body1, 0, contactBreakingThreshold, contactProcessingThreshold);
	else
	{
		// Softbody always first
		btSoftBody* temp = (btSoftBody*)body0;
		manifold = new (mem) btPersistentManifold(body0, body1, 0, contactBreakingThreshold, contactProcessingThreshold, temp->m_nodes.size());
	}
	if (!m_batchUpdating)
	{
		// batch updater will update manifold pointers array after finishing, so
		// only need to update array when not batch-updating
		//btAssert( !btThreadsAreRunning() );
		manifold->m_index1a = m_manifoldsPtr.size();
		m_manifoldsPtr.push_back(manifold);
	}
	else
	{
		m_batchManifoldsPtr[btGetCurrentThreadIndex()].push_back(manifold);
	}

	return manifold;
}

void btCollisionDispatcherMt::releaseManifold(btPersistentManifold* manifold)
{
	//btAssert( !btThreadsAreRunning() );
	
	if (!m_batchUpdating)
	{
		clearManifold(manifold);
		// batch updater will update manifold pointers array after finishing, so
		// only need to update array when not batch-updating
		int findIndex = manifold->m_index1a;
		btAssert(findIndex < m_manifoldsPtr.size());
		m_manifoldsPtr.swap(findIndex, m_manifoldsPtr.size() - 1);
		m_manifoldsPtr[findIndex]->m_index1a = findIndex;
		m_manifoldsPtr[findIndex]->m_hasCollided = false;
		m_manifoldsPtr.pop_back();
	} else {
		m_batchReleasePtr[btGetCurrentThreadIndex()].push_back(manifold);
		return;
	}

	manifold->freeContactPoint();
	if (m_persistentManifoldPoolAllocator->validPtr(manifold))
	{
		m_persistentManifoldPoolAllocator->freeMemory(manifold);
	}
	else
	{
		btAlignedFree(manifold);
	}
}

void btCollisionDispatcherMt::releaseCachedManifold(btPersistentManifold* manifold)
{
	//btAssert( !btThreadsAreRunning() );
	
	// if (!m_batchUpdating)
	// {
		// batch updater will update manifold pointers array after finishing, so
		// only need to update array when not batch-updating
		// int findIndex = manifold->m_index1a;
		// btAssert(findIndex < m_collidedManifoldsCache.size());
		// m_collidedManifoldsCache.swap(findIndex, m_collidedManifoldsCache.size() - 1);
		// m_collidedManifoldsCache[findIndex]->m_index1a = findIndex;
		// m_collidedManifoldsCache.pop_back();
	// } else {
	// 	m_batchReleasePtr[btGetCurrentThreadIndex()].push_back(manifold);
	// 	return;
	// }

	manifold->freeContactPoint();
	free(manifold);
}


void btCollisionDispatcherMt::addManifoldToCache(btPersistentManifold* manifold)
{
	for (int i = 0; i < m_collidedManifoldsCache.size(); i++)
	{
		bool isAlreadyInCache = m_collidedManifoldsCache.at(i)->getBody0() == manifold->getBody0() || m_collidedManifoldsCache.at(i)->getBody0() == manifold->getBody1();
		isAlreadyInCache = isAlreadyInCache && (m_collidedManifoldsCache.at(i)->getBody1() == manifold->getBody0() || m_collidedManifoldsCache.at(i)->getBody1() == manifold->getBody1());
		
		if (isAlreadyInCache)
		{
			m_collidedManifoldsCache.removeAtIndex(i);
			break;
		}
	}
	m_collidedManifoldsCache.push_back(manifold);
}

void btCollisionDispatcherMt::ClearManifoldsCache()
{
	releaseAllCachedManifolds();
	m_collidedManifoldsCache.clear();
}

void btCollisionDispatcherMt::releaseAllCachedManifolds()
{
	//btAssert( !btThreadsAreRunning() );

	while (	m_collidedManifoldsCache.size() > 0)
	{
		btPersistentManifold* manifold = m_collidedManifoldsCache[m_collidedManifoldsCache.size()-1];
		m_collidedManifoldsCache.pop_back();

		manifold->freeContactPoint();
		if (m_persistentManifoldPoolAllocator->validPtr(manifold))
		{
			m_persistentManifoldPoolAllocator->freeMemory(manifold);
		}
		else
		{
			btAlignedFree(manifold);
		}
	}
	
}


int btCollisionDispatcherMt::getNumManifoldsCache() const
{
	return int(m_collidedManifoldsCache.size());
}

btPersistentManifold* btCollisionDispatcherMt::getManifoldsCacheByIndexInternal(int index)
{
	btAssert(index>=0);
	btAssert(index<m_collidedManifoldsCache.size());
	return m_collidedManifoldsCache[index];
}

struct CollisionDispatcherUpdater : public btIParallelForBody
{
	btBroadphasePair* mPairArray;
	btNearCallback mCallback;
	btCollisionDispatcher* mDispatcher;
	const btDispatcherInfo* mInfo;

	CollisionDispatcherUpdater()
	{
		mPairArray = NULL;
		mCallback = NULL;
		mDispatcher = NULL;
		mInfo = NULL;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btBroadphasePair* pair = &mPairArray[i];
			mCallback(*pair, *mDispatcher, *mInfo);
		}
	}
};

void btCollisionDispatcherMt::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& info, btDispatcher* dispatcher)
{
	const int pairCount = pairCache->getNumOverlappingPairs();
	if (pairCount == 0)
	{
		return;
	}
	CollisionDispatcherUpdater updater;
	updater.mCallback = getNearCallback();
	updater.mPairArray = pairCache->getOverlappingPairArrayPtr();
	updater.mDispatcher = this;
	updater.mInfo = &info;

	m_batchUpdating = true;
	btParallelFor(0, pairCount, m_grainSize, updater);
	m_batchUpdating = false;

	// merge new manifolds, if any
	for (int i = 0; i < m_batchManifoldsPtr.size(); ++i)
	{
		btAlignedObjectArray<btPersistentManifold*>& batchManifoldsPtr = m_batchManifoldsPtr[i];

		for (int j = 0; j < batchManifoldsPtr.size(); ++j)
		{
			batchManifoldsPtr[j]->m_index1a = m_manifoldsPtr.size();
			m_manifoldsPtr.push_back(batchManifoldsPtr[j]);
		}

		batchManifoldsPtr.resizeNoInitialize(0);
	}

	// remove batched remove manifolds.
	for (int i = 0; i < m_batchReleasePtr.size(); ++i)
	{
		btAlignedObjectArray<btPersistentManifold*>& batchManifoldsPtr = m_batchReleasePtr[i];
		for (int j = 0; j < batchManifoldsPtr.size(); ++j)
		{
			releaseManifold(batchManifoldsPtr[j]);
		}
		batchManifoldsPtr.resizeNoInitialize(0);
	}

	// update the indices (used when releasing manifolds)
	for (int i = 0; i < m_manifoldsPtr.size(); ++i)
	{
		m_manifoldsPtr[i]->m_index1a = i;
	}
}
