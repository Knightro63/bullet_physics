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

import "package:bullet_physics/collision/broadphase/collision_algorithm.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/utils/object_array_list.dart";

abstract class Dispatcher {
	CollisionAlgorithm? findAlgorithm([CollisionObject? body0, CollisionObject? body1, PersistentManifold? sharedManifold]);
	PersistentManifold getNewManifold(Object? body0, Object? body1);
	void releaseManifold(PersistentManifold? manifold);
	void clearManifold(PersistentManifold? manifold);
	bool needsCollision(CollisionObject? body0, CollisionObject? body1);
	bool needsResponse(CollisionObject body0, CollisionObject body1);
	void dispatchAllCollisionPairs(OverlappingPairCache? pairCache, DispatcherInfo dispatchInfo, Dispatcher? dispatcher);
	int getNumManifolds();
	PersistentManifold? getManifoldByIndexInternal(int index);
	ObjectArrayList<PersistentManifold> getInternalManifoldPointer();
	void freeCollisionAlgorithm(CollisionAlgorithm? algo);
}
