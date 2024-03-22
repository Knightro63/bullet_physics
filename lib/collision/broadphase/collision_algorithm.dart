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

import "package:bullet_physics/collision/broadphase/collision_algorithm_construction_info.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/collision_algorithm_create_func.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/utils/object_array_list.dart";

/**
 * Collision algorithm for handling narrowphase or midphase collision detection
 * between two collision object types.
 * 
 * @author jezek2
 */
abstract class CollisionAlgorithm {
	//final BulletStack stack = BulletStack.get();
	CollisionAlgorithmCreateFunc? _createFunc;
	Dispatcher? dispatcher;

  CollisionAlgorithm([CollisionAlgorithmConstructionInfo? ci]){
    initCA(ci);
  }

	void initCA(CollisionAlgorithmConstructionInfo? ci) {
		dispatcher = ci?.dispatcher1;
	}
	
	void destroy();
	void processCollision(CollisionObject? body0, CollisionObject? body1, DispatcherInfo? dispatchInfo, ManifoldResult? resultOut);
	double calculateTimeOfImpact(CollisionObject? body0, CollisionObject? body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut);
	void getAllContactManifolds(ObjectArrayList<PersistentManifold> manifoldArray);
	void internalSetCreateFunc(CollisionAlgorithmCreateFunc? func) {
		_createFunc = func;
	}

	CollisionAlgorithmCreateFunc? internalGetCreateFunc() {
		return _createFunc;
	}
}
