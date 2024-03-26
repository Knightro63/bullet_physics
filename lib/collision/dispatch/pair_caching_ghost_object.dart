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

import "package:bullet_physics/collision/broadphase/broadphase_proxy.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/hashed_overlapping_pair_cache.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/ghost_object.dart";

class PairCachingGhostObject extends GhostObject {
	HashedOverlappingPairCache hashPairCache = HashedOverlappingPairCache();

	/**
	 * This method is mainly for expert/internal use only.
	 */
	@override
	void addOverlappingObjectInternal(BroadphaseProxy otherProxy, BroadphaseProxy? thisProxy) {
		BroadphaseProxy? actualThisProxy = thisProxy ?? getBroadphaseHandle();
		assert(actualThisProxy != null);

		CollisionObject? otherObject = otherProxy.clientObject as CollisionObject?;
		assert (otherObject != null);

		// if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
		int index = overlappingObjects.indexOf(otherObject!);
		if (index == -1) {
			overlappingObjects.add(otherObject);
			hashPairCache.addOverlappingPair(actualThisProxy!, otherProxy);
		}
	}

	@override
	void removeOverlappingObjectInternal(BroadphaseProxy? otherProxy, Dispatcher? dispatcher, BroadphaseProxy? thisProxy) {
		CollisionObject? otherObject = otherProxy?.clientObject as CollisionObject?;
		BroadphaseProxy? actualThisProxy = thisProxy ?? getBroadphaseHandle();
		assert(actualThisProxy != null);

		assert (otherObject != null);
		int index = overlappingObjects.indexOf(otherObject!);
		if (index != -1) {
			overlappingObjects.setQuick(index, overlappingObjects.getQuick(overlappingObjects.size-1));
			overlappingObjects.removeAt(overlappingObjects.size-1);
			hashPairCache.removeOverlappingPair(actualThisProxy!, otherProxy, dispatcher);
		}
	}

	HashedOverlappingPairCache getOverlappingPairCache() {
		return hashPairCache;
	}
}
