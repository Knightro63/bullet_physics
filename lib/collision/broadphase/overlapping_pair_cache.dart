/*
 * Dart port of Bullet (c) 2024 @Knightro63
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

import "package:bullet_physics/collision/broadphase/broadphase_pair.dart";
import "package:bullet_physics/collision/broadphase/broadphase_proxy.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/overlap_callback.dart";
import "package:bullet_physics/collision/broadphase/overlap_filter_callback.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_callback.dart";
import "package:bullet_physics/utils/object_array_list.dart";

/**
 * OverlappingPairCache provides an interface for overlapping pair management (add,
 * remove, storage), used by the {@link BroadphaseInterface} broadphases.
 *
 * @author jezek2
 */
abstract class OverlappingPairCache extends OverlappingPairCallback {
	ObjectArrayList<BroadphasePair?> getOverlappingPairArray();
	void cleanOverlappingPair(BroadphasePair? pair, Dispatcher? dispatcher);
	int getNumOverlappingPairs();
	void cleanProxyFromPairs(BroadphaseProxy proxy, Dispatcher? dispatcher);
	void setOverlapFilterCallback(OverlapFilterCallback overlapFilterCallback);
	void processAllOverlappingPairs(OverlapCallback callback, Dispatcher? dispatcher);
	BroadphasePair? findPair(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1);
	bool hasDeferredRemoval();
	void setInternalGhostPairCallback(OverlappingPairCallback ghostPairCallback);
}
