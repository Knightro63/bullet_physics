/*
 * Dart port of Bullet (c) 2024 @Knightro
 * 
 * AxisSweep3
 * Copyright (c) 2006 Simon Hobbs
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import "package:bullet_physics/collision/broadphase/broadphase_interface.dart";
import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/broadphase/broadphase_pair.dart";
import "package:bullet_physics/collision/broadphase/broadphase_proxy.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/hashed_overlapping_pair_cache.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_callback.dart";
import "package:bullet_physics/core/bullet_stats.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

abstract class AxisSweep3Internal extends BroadphaseInterface {
	EdgeArray createEdgeArray(int size);
	Handle createHandle();
	int getMask();
  
	int bpHandleMask = 0;
	int handleSentinel = 0;
	
	final Vector3 worldAabbMin = Vector3.zero(); // overall system bounds
	final Vector3 worldAabbMax = Vector3.zero(); // overall system bounds

	final Vector3 quant = Vector3.zero();     // scaling factor for quantization

	int numHandles = 0;                               // number of active handles
	int maxHandles = 0;                               // max number of handles
	List<Handle> pHandles = [];                            // handles pool
	int firstFreeHandle = 0;		                    // free handles list

	List<EdgeArray?> pEdges = [null,null,null];      // edge arrays for the 3 axes (each array has m_maxHandles * 2 + 2 sentinel entries)

	late OverlappingPairCache pairCache;
	
	// OverlappingPairCallback is an additional optional user callback for adding/removing overlapping pairs, similar interface to OverlappingPairCache.
	OverlappingPairCallback? userPairCallback;
	
	bool ownsPairCache = false;

	int invalidPair = 0;
	
	// JAVA NOTE: added
	int mask = 0;
	
	AxisSweep3Internal(Vector3 worldAabbMin, Vector3 worldAabbMax, this.bpHandleMask, this.handleSentinel, int userMaxHandles, OverlappingPairCache? pairCache) {
    this.pairCache = pairCache ?? HashedOverlappingPairCache();

		int maxHandles = userMaxHandles + 1; // need to add one sentinel handle

		if (pairCache == null) {
			ownsPairCache = true;
		}

		//assert(bounds.HasVolume());

		// init bounds
		this.worldAabbMin.setFrom(worldAabbMin);
		this.worldAabbMax.setFrom(worldAabbMax);

		Vector3 aabbSize = Vector3.zero();
		aabbSize.sub2(this.worldAabbMax,this.worldAabbMin);

		int maxInt = handleSentinel;

		quant.setValues(maxInt / aabbSize.x, maxInt / aabbSize.y, maxInt / aabbSize.z);

		// allocate handles buffer and put all handles on free list
		for (int i=0; i< maxHandles; i++) {
			pHandles.add(createHandle());
		}
		this.maxHandles = maxHandles;
		numHandles = 0;

		// handle 0 is reserved as the null index, and is also used as the sentinel
		firstFreeHandle = 1;
		
    for (int i = firstFreeHandle; i<maxHandles; i++) {
      pHandles[i].setNextFree(i+1);
    }
    pHandles[maxHandles - 1].setNextFree(0);

    for (int i=0; i<3; i++) {
      pEdges[i] = createEdgeArray(maxHandles*2);
    }

		pHandles[0].clientObject = null;

		for (int axis = 0; axis < 3; axis++) {
			pHandles[0].setMinEdges(axis, 0);
			pHandles[0].setMaxEdges(axis, 1);

			pEdges[axis]?.setPos(0, 0);
			pEdges[axis]?.setHandle(0, 0);
			pEdges[axis]?.setPos(1, handleSentinel);
			pEdges[axis]?.setHandle(1, 0);
		}
		
		mask = getMask();
	}

	// allocation/deallocation
	int allocHandle() {
		assert (firstFreeHandle != 0);

		int handle = firstFreeHandle;
		firstFreeHandle = getHandle(handle).getNextFree();
		numHandles++;

		return handle;
	}
	
	void freeHandle(int handle) {
		assert (handle > 0 && handle < maxHandles);

		getHandle(handle).setNextFree(firstFreeHandle);
		firstFreeHandle = handle;

		numHandles--;
	}
	
	bool testOverlap(int ignoreAxis, Handle pHandleA, Handle pHandleB) {
		// optimization 1: check the array index (memory address), instead of the m_pos

		for (int axis=0; axis<3; axis++) {
			if (axis != ignoreAxis) {
				if (pHandleA.getMaxEdges(axis) < pHandleB.getMinEdges(axis) || pHandleB.getMaxEdges(axis) < pHandleA.getMinEdges(axis)) {
					return false;
				}
			}
		}

		return true;
	}

	void quantize(List<int> out, Vector3 point, int isMax) {
		Vector3 clampedPoint = Vector3.copy(point);

		VectorUtil.setMax(clampedPoint, worldAabbMin);
		VectorUtil.setMin(clampedPoint, worldAabbMax);

		Vector3 v = Vector3.zero();
		v.sub2(clampedPoint, worldAabbMin);
		VectorUtil.mul(v, v, quant);

		out[0] = ((v.x.toInt() & bpHandleMask) | isMax) & mask;
		out[1] = ((v.y.toInt() & bpHandleMask) | isMax) & mask;
		out[2] = ((v.z.toInt() & bpHandleMask) | isMax) & mask;
	}

	// sorting a min edge downwards can only ever *add* overlaps
	void sortMinDown(int axis, int edge, Dispatcher? dispatcher, bool updateOverlaps) {
		EdgeArray edgeArray = pEdges[axis]!;
		int pEdgeIdx = edge;
		int pPrevIdx = pEdgeIdx - 1;

		Handle pHandleEdge = getHandle(edgeArray.getHandle(pEdgeIdx));

		while (edgeArray.getPos(pEdgeIdx) < edgeArray.getPos(pPrevIdx)) {
			Handle pHandlePrev = getHandle(edgeArray.getHandle(pPrevIdx));

			if (edgeArray.isMax(pPrevIdx) != 0) {
				// if previous edge is a maximum check the bounds and add an overlap if necessary
				if (updateOverlaps && testOverlap(axis, pHandleEdge, pHandlePrev)) {
					pairCache.addOverlappingPair(pHandleEdge, pHandlePrev);
					if (userPairCallback != null) {
						userPairCallback?.addOverlappingPair(pHandleEdge, pHandlePrev);
					}
				}
				pHandlePrev.incMaxEdges(axis);
			}
			else {
				pHandlePrev.incMinEdges(axis);
			}
			pHandleEdge.decMinEdges(axis);

			// swap the edges
			edgeArray.swap(pEdgeIdx, pPrevIdx);

			// decrement
			pEdgeIdx--;
			pPrevIdx--;
		}
	}
	
	// sorting a min edge upwards can only ever *remove* overlaps
	void sortMinUp(int axis, int edge, Dispatcher? dispatcher, bool updateOverlaps) {
		EdgeArray edgeArray = pEdges[axis]!;
		int pEdgeIdx = edge;
		int pNextIdx = pEdgeIdx + 1;
		Handle pHandleEdge = getHandle(edgeArray.getHandle(pEdgeIdx));

		while (edgeArray.getHandle(pNextIdx) != 0 && (edgeArray.getPos(pEdgeIdx) >= edgeArray.getPos(pNextIdx))) {
			Handle pHandleNext = getHandle(edgeArray.getHandle(pNextIdx));

			if (edgeArray.isMax(pNextIdx) != 0) {
				// if next edge is maximum remove any overlap between the two handles
				if (updateOverlaps) {
					Handle handle0 = getHandle(edgeArray.getHandle(pEdgeIdx));
					Handle handle1 = getHandle(edgeArray.getHandle(pNextIdx));

					pairCache.removeOverlappingPair(handle0, handle1, dispatcher);
					userPairCallback?.removeOverlappingPair(handle0, handle1, dispatcher);
				}

				// update edge reference in other handle
				pHandleNext.decMaxEdges(axis);
			}
			else {
				pHandleNext.decMinEdges(axis);
			}
			pHandleEdge.incMinEdges(axis);

			// swap the edges
			edgeArray.swap(pEdgeIdx, pNextIdx);

			// increment
			pEdgeIdx++;
			pNextIdx++;
		}
	}
	
	// sorting a max edge downwards can only ever *remove* overlaps
	void sortMaxDown(int axis, int edge, Dispatcher? dispatcher, bool updateOverlaps) {
		EdgeArray edgeArray = pEdges[axis]!;
		int pEdgeIdx = edge;
		int pPrevIdx = pEdgeIdx - 1;
		Handle pHandleEdge = getHandle(edgeArray.getHandle(pEdgeIdx));

		while (edgeArray.getPos(pEdgeIdx) < edgeArray.getPos(pPrevIdx)) {
			Handle pHandlePrev = getHandle(edgeArray.getHandle(pPrevIdx));

			if (edgeArray.isMax(pPrevIdx) == 0) {
				// if previous edge was a minimum remove any overlap between the two handles
				if (updateOverlaps) {
					// this is done during the overlappingpairarray iteration/narrowphase collision
					Handle handle0 = getHandle(edgeArray.getHandle(pEdgeIdx));
					Handle handle1 = getHandle(edgeArray.getHandle(pPrevIdx));
					pairCache.removeOverlappingPair(handle0, handle1, dispatcher);
					if (userPairCallback != null) {
						userPairCallback?.removeOverlappingPair(handle0, handle1, dispatcher);
					}
				}

				// update edge reference in other handle
				pHandlePrev.incMinEdges(axis);
			}
			else {
				pHandlePrev.incMaxEdges(axis);
			}
			pHandleEdge.decMaxEdges(axis);

			// swap the edges
			edgeArray.swap(pEdgeIdx, pPrevIdx);

			// decrement
			pEdgeIdx--;
			pPrevIdx--;
		}
	}
	
	// sorting a max edge upwards can only ever *add* overlaps
	void sortMaxUp(int axis, int edge, Dispatcher? dispatcher, bool updateOverlaps) {
		EdgeArray edgeArray = pEdges[axis]!;
		int pEdgeIdx = edge;
		int pNextIdx = pEdgeIdx + 1;
		Handle pHandleEdge = getHandle(edgeArray.getHandle(pEdgeIdx));

		while (edgeArray.getHandle(pNextIdx) != 0 && (edgeArray.getPos(pEdgeIdx) >= edgeArray.getPos(pNextIdx))) {
			Handle pHandleNext = getHandle(edgeArray.getHandle(pNextIdx));

			if (edgeArray.isMax(pNextIdx) == 0) {
				// if next edge is a minimum check the bounds and add an overlap if necessary
				if (updateOverlaps && testOverlap(axis, pHandleEdge, pHandleNext)) {
					Handle handle0 = getHandle(edgeArray.getHandle(pEdgeIdx));
					Handle handle1 = getHandle(edgeArray.getHandle(pNextIdx));
					pairCache.addOverlappingPair(handle0, handle1);
					if (userPairCallback != null) {
						userPairCallback?.addOverlappingPair(handle0, handle1);
					}
				}

				// update edge reference in other handle
				pHandleNext.decMinEdges(axis);
			}
			else {
				pHandleNext.decMaxEdges(axis);
			}
			pHandleEdge.incMaxEdges(axis);

			// swap the edges
			edgeArray.swap(pEdgeIdx, pNextIdx);

			// increment
			pEdgeIdx++;
			pNextIdx++;
		}
	}
	
	int getNumHandles() {
		return numHandles;
	}
  @override
	void calculateOverlappingPairs(Dispatcher? dispatcher) {
		if (pairCache.hasDeferredRemoval()) {
			ObjectArrayList<BroadphasePair?> overlappingPairArray = pairCache.getOverlappingPairArray();

			// perform a sort, to find duplicates and to sort 'invalid' pairs to the end
			MiscUtil.quickSortObjectArray(overlappingPairArray, BroadphasePair.broadphasePairSortPredicate);
			MiscUtil.resizeObjectArray(overlappingPairArray, overlappingPairArray.size - invalidPair, BroadphasePair);
			invalidPair = 0;

			BroadphasePair previousPair = BroadphasePair();
			previousPair.pProxy0 = null;
			previousPair.pProxy1 = null;
			previousPair.algorithm = null;

			for (int i=0; i<overlappingPairArray.size; i++) {
				BroadphasePair? pair = overlappingPairArray.getQuick(i);

				bool isDuplicate = (pair?.equals(previousPair) ?? false);

				previousPair.set(pair);

				bool needsRemoval = false;

				if (!isDuplicate) {
					bool hasOverlap = testAabbOverlap(pair?.pProxy0, pair?.pProxy1);

					if (hasOverlap) {
						needsRemoval = false;
					}
					else {
						needsRemoval = true;
					}
				}
				else {
					// remove duplicate
					needsRemoval = true;
					// should have no algorithm
					assert (pair?.algorithm == null);
				}

				if (needsRemoval) {
					pairCache.cleanOverlappingPair(pair, dispatcher);
					pair?.pProxy0 = null;
					pair?.pProxy1 = null;
					invalidPair++;
					BulletStats.gOverlappingPairs--;
				}

			}

			MiscUtil.quickSortObjectArray(overlappingPairArray, BroadphasePair.broadphasePairSortPredicate);
			MiscUtil.resizeObjectArray(overlappingPairArray, overlappingPairArray.size - invalidPair, BroadphasePair);
			invalidPair = 0;
		}
	}
	
	int addHandle(Vector3 aabbMin, Vector3 aabbMax, Object pOwner, int collisionFilterGroup, int collisionFilterMask, Dispatcher? dispatcher, Object? multiSapProxy) {
		// quantize the bounds
		List<int> min = [0,0,0], max = [0,0,0];
		quantize(min, aabbMin, 0);
		quantize(max, aabbMax, 1);

		// allocate a handle
		int handle = allocHandle();

		Handle pHandle = getHandle(handle);

		pHandle.uniqueId = handle;
		pHandle.clientObject = pOwner;
		pHandle.collisionFilterGroup = collisionFilterGroup;
		pHandle.collisionFilterMask = collisionFilterMask;
		pHandle.multiSapParentProxy = multiSapProxy;

		// compute current limit of edge arrays
		int limit = numHandles * 2;

		// insert edges just inside the max boundary edge
		for (int axis = 0; axis < 3; axis++) {
			pHandles[0].setMaxEdges(axis, pHandles[0].getMaxEdges(axis) + 2);

			pEdges[axis]?.set(limit + 1, limit - 1);

			pEdges[axis]?.setPos(limit - 1, min[axis]);
			pEdges[axis]?.setHandle(limit - 1, handle);

			pEdges[axis]?.setPos(limit, max[axis]);
			pEdges[axis]?.setHandle(limit, handle);

			pHandle.setMinEdges(axis, limit - 1);
			pHandle.setMaxEdges(axis, limit);
		}

		// now sort the edges to their correct position
		sortMinDown(0, pHandle.getMinEdges(0), dispatcher, false);
		sortMaxDown(0, pHandle.getMaxEdges(0), dispatcher, false);
		sortMinDown(1, pHandle.getMinEdges(1), dispatcher, false);
		sortMaxDown(1, pHandle.getMaxEdges(1), dispatcher, false);
		sortMinDown(2, pHandle.getMinEdges(2), dispatcher, true);
		sortMaxDown(2, pHandle.getMaxEdges(2), dispatcher, true);

		return handle;
	}
	
	void removeHandle(int handle, Dispatcher? dispatcher) {
		Handle pHandle = getHandle(handle);

		// explicitly remove the pairs containing the proxy
		// we could do it also in the sortMinUp (passing true)
		// todo: compare performance
		if (!pairCache.hasDeferredRemoval()) {
			pairCache.removeOverlappingPairsContainingProxy(pHandle, dispatcher);
		}

		// compute current limit of edge arrays
		int limit = numHandles * 2;

		for (int axis = 0; axis < 3; axis++) {
			pHandles[0].setMaxEdges(axis, pHandles[0].getMaxEdges(axis) - 2);
		}

		// remove the edges by sorting them up to the end of the list
		for (int axis = 0; axis < 3; axis++) {
			EdgeArray pEdges = this.pEdges[axis]!;
			int max = pHandle.getMaxEdges(axis);
			pEdges.setPos(max, handleSentinel);

			sortMaxUp(axis, max, dispatcher, false);

			int i = pHandle.getMinEdges(axis);
			pEdges.setPos(i, handleSentinel);

			sortMinUp(axis, i, dispatcher, false);

			pEdges.setHandle(limit - 1, 0);
			pEdges.setPos(limit - 1, handleSentinel);
		}

		// free the handle
		freeHandle(handle);
	}
	
	void updateHandle(int handle, Vector3 aabbMin, Vector3 aabbMax, Dispatcher? dispatcher) {
		Handle pHandle = getHandle(handle);

		// quantize the bounds
		List<int> min = [0,0,0], max = [0,0,0];
		quantize(min, aabbMin, 0);
		quantize(max, aabbMax, 1);

		// update changed edges
		for (int axis = 0; axis < 3; axis++) {
			int emin = pHandle.getMinEdges(axis);
			int emax = pHandle.getMaxEdges(axis);

			int dmin = min[axis] - pEdges[axis]!.getPos(emin);
			int dmax = max[axis] - pEdges[axis]!.getPos(emax);

			pEdges[axis]?.setPos(emin, min[axis]);
			pEdges[axis]?.setPos(emax, max[axis]);

			// expand (only adds overlaps)
			if (dmin < 0) {
				sortMinDown(axis, emin, dispatcher, true);
			}
			if (dmax > 0) {
				sortMaxUp(axis, emax, dispatcher, true); // shrink (only removes overlaps)
			}
			if (dmin > 0) {
				sortMinUp(axis, emin, dispatcher, true);
			}
			if (dmax < 0) {
				sortMaxDown(axis, emax, dispatcher, true);
			}
		}
	}
	
	Handle getHandle(int index) {
		return pHandles[index];
	}
	
  @override
	BroadphaseProxy createProxy(Vector3 aabbMin, Vector3 aabbMax, BroadphaseNativeType shapeType, Object userPtr, int collisionFilterGroup, int collisionFilterMask, Dispatcher? dispatcher, Object? multiSapProxy) {
		int handleId = addHandle(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher, multiSapProxy);

		Handle handle = getHandle(handleId);

		return handle;
	}
  @override
	void destroyProxy(BroadphaseProxy proxy, Dispatcher? dispatcher) {
		Handle handle = proxy as Handle;
		removeHandle(handle.uniqueId, dispatcher);
	}
  @override
	void setAabb(BroadphaseProxy? proxy, Vector3 aabbMin, Vector3 aabbMax, Dispatcher? dispatcher) {
		Handle handle = proxy as Handle;
		updateHandle(handle.uniqueId, aabbMin, aabbMax, dispatcher);
	}
	
	bool testAabbOverlap(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1) {
    if(proxy0 == null || proxy1 == null) return false;
		Handle pHandleA = proxy0 as Handle;
		Handle pHandleB = proxy1 as Handle;

		// optimization 1: check the array index (memory address), instead of the m_pos

		for (int axis = 0; axis < 3; axis++) {
			if (pHandleA.getMaxEdges(axis) < pHandleB.getMinEdges(axis) ||
					pHandleB.getMaxEdges(axis) < pHandleA.getMinEdges(axis)) {
				return false;
			}
		}
		return true;
	}
  @override
	OverlappingPairCache getOverlappingPairCache() {
		return pairCache;
	}

	void setOverlappingPairUserCallback(OverlappingPairCallback pairCallback) {
		userPairCallback = pairCallback;
	}
	
	OverlappingPairCallback? getOverlappingPairUserCallback() {
		return userPairCallback;
	}
	
	// getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	// will add some transform later
  @override
	void getBroadphaseAabb(Vector3 aabbMin, Vector3 aabbMax) {
		aabbMin.setFrom(worldAabbMin);
		aabbMax.setFrom(worldAabbMax);
	}
  @override
	void printStats() {
		/*
		printf("btAxisSweep3.h\n");
		printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
		printf("aabbMin=%f,%f,%f,aabbMax=%f,%f,%f\n",m_worldAabbMin.getX(),m_worldAabbMin.getY(),m_worldAabbMin.getZ(),
		m_worldAabbMax.getX(),m_worldAabbMax.getY(),m_worldAabbMax.getZ());
		*/
	}
	
	////////////////////////////////////////////////////////////////////////////
}

// abstract EdgeArray createEdgeArray(int size);
// abstract Handle createHandle();
// abstract int getMask();

abstract class EdgeArray {
  void swap(int idx1, int idx2);
  void set(int dest, int src);
  
  int getPos(int index);
  void setPos(int index, int value);

  int getHandle(int index);
  void setHandle(int index, int value);
  
  int isMax(int offset) {
    return (getPos(offset) & 1);
  }
}

abstract class Handle extends BroadphaseProxy {
  int getMinEdges(int edgeIndex);
  void setMinEdges(int edgeIndex, int value);
  
  int getMaxEdges(int edgeIndex);
  void setMaxEdges(int edgeIndex, int value);

  void incMinEdges(int edgeIndex) {
    setMinEdges(edgeIndex, getMinEdges(edgeIndex)+1);
  }

  void incMaxEdges(int edgeIndex) {
    setMaxEdges(edgeIndex, getMaxEdges(edgeIndex)+1);
  }

  void decMinEdges(int edgeIndex) {
    setMinEdges(edgeIndex, getMinEdges(edgeIndex)-1);
  }

  void decMaxEdges(int edgeIndex) {
    setMaxEdges(edgeIndex, getMaxEdges(edgeIndex)-1);
  }
  
  void setNextFree(int next) {
    setMinEdges(0, next);
  }
  
  int getNextFree() {
    return getMinEdges(0);
  }
}
