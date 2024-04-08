/*
 * Dart port of Bullet (c) 2024 @Knightro
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

import 'package:bullet_physics/collision/broadphase/broadphase_interface.dart';
import 'package:bullet_physics/collision/broadphase/broadphase_native_type.dart';
import 'package:bullet_physics/collision/broadphase/broadphase_proxy.dart';
import 'package:bullet_physics/collision/broadphase/dispatcher.dart';
import 'package:bullet_physics/collision/broadphase/hashed_overlapping_pair_cache.dart';
import 'package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart';
import 'package:bullet_physics/collision/broadphase/simple_broadphase_proxy.dart';
import 'package:bullet_physics/utils/object_array_list.dart';
import 'package:vector_math/vector_math.dart';

class SimpleBroadphase extends BroadphaseInterface {
	final ObjectArrayList<SimpleBroadphaseProxy> _handles = ObjectArrayList();
	late OverlappingPairCache _pairCache;
	bool ownsPairCache = false;
	
	SimpleBroadphase([int maxProxies = 16384, OverlappingPairCache? overlappingPairCache]) {
		_pairCache = overlappingPairCache ?? HashedOverlappingPairCache();

		if (overlappingPairCache == null) {
			ownsPairCache = true;
		}
	}

  @override
	BroadphaseProxy createProxy(
    Vector3 aabbMin, 
    Vector3 aabbMax, 
    BroadphaseNativeType shapeType, 
    Object userPtr, 
    int collisionFilterGroup, 
    int collisionFilterMask, 
    Dispatcher? dispatcher, 
    Object? multiSapProxy
  ) {
		assert (aabbMin.x <= aabbMax.x && aabbMin.y <= aabbMax.y && aabbMin.z <= aabbMax.z);

		SimpleBroadphaseProxy proxy = SimpleBroadphaseProxy(aabbMin, aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask, multiSapProxy);
		proxy.uniqueId = _handles.size;
		_handles.add(proxy);
		return proxy;
	}

  @override
	void destroyProxy(BroadphaseProxy proxy, Dispatcher? dispatcher) {
		_handles.remove(proxy as SimpleBroadphaseProxy);
		_pairCache.removeOverlappingPairsContainingProxy(proxy, dispatcher);
	}

  @override
	void setAabb(BroadphaseProxy? proxy, Vector3 aabbMin, Vector3 aabbMax, Dispatcher? dispatcher) {
		SimpleBroadphaseProxy sbp = proxy as SimpleBroadphaseProxy;
		sbp.min.setFrom(aabbMin);
		sbp.max.setFrom(aabbMax);
	}

	static bool _aabbOverlap(SimpleBroadphaseProxy proxy0, SimpleBroadphaseProxy proxy1) {
		return proxy0.min.x <= proxy1.max.x && proxy1.min.x <= proxy0.max.x &&
				proxy0.min.y <= proxy1.max.y && proxy1.min.y <= proxy0.max.y &&
				proxy0.min.z <= proxy1.max.z && proxy1.min.z <= proxy0.max.z;
	}

  @override
	void calculateOverlappingPairs(Dispatcher? dispatcher) {
		for (int i=0; i<_handles.size; i++) {
			SimpleBroadphaseProxy? proxy0 = _handles.getQuick(i);
			for (int j=0; j< _handles.size; j++) {
				SimpleBroadphaseProxy? proxy1 = _handles.getQuick(j);
				if (proxy0 == proxy1) continue;
				
				if (_aabbOverlap(proxy0!, proxy1!)) {
					if (_pairCache.findPair(proxy0, proxy1) == null) {
						_pairCache.addOverlappingPair(proxy0, proxy1);
					}
				}
				else {
					if (!_pairCache.hasDeferredRemoval()) {
						if (_pairCache.findPair(proxy0, proxy1) != null) {
							_pairCache.removeOverlappingPair(proxy0, proxy1, dispatcher);
						}
					}
				}
			}
		}
	}

  @override
	OverlappingPairCache getOverlappingPairCache() {
		return _pairCache;
	}

  @override
	void getBroadphaseAabb(Vector3 aabbMin, Vector3 aabbMax) {
		aabbMin.setValues(-1e30, -1e30, -1e30);
		aabbMax.setValues(1e30, 1e30, 1e30);
	}

  @override
	void printStats() {
    //		System.out.printf("btSimpleBroadphase.h\n");
    //		System.out.printf("numHandles = %d, maxHandles = %d\n", /*numHandles*/ handles.length, maxHandles);
	}
	
}
