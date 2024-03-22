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
import "package:bullet_physics/collision/broadphase/dispatch_func.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/collision_dispatcher.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/dispatch/near_callback.dart";

/**
 * Default implementation of {@link NearCallback}.
 * 
 * @author jezek2
 */
class DefaultNearCallback extends NearCallback {
	final ManifoldResult _contactPointResult = ManifoldResult();

  @override
	void handleCollision(BroadphasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo) {
		CollisionObject? colObj0 = collisionPair.pProxy0?.clientObject as CollisionObject?;
		CollisionObject? colObj1 = collisionPair.pProxy1?.clientObject as CollisionObject?;

		if (dispatcher.needsCollision(colObj0, colObj1)) {
			// dispatcher will keep algorithms persistent in the collision pair
			collisionPair.algorithm ??= dispatcher.findAlgorithm(colObj0, colObj1);

			if (collisionPair.algorithm != null) {
				//ManifoldResult contactPointResult = ManifoldResult(colObj0, colObj1);
				_contactPointResult.init(colObj0, colObj1);

				if (dispatchInfo.dispatchFunc == DispatchFunc.discrete) {
					// discrete collision detection query
					collisionPair.algorithm?.processCollision(colObj0, colObj1, dispatchInfo, _contactPointResult);
				}
				else {
					// continuous collision detection query, time of impact (toi)
					double toi = collisionPair.algorithm?.calculateTimeOfImpact(colObj0, colObj1, dispatchInfo, _contactPointResult) ?? 0;
					if (dispatchInfo.timeOfImpact > toi) {
						dispatchInfo.timeOfImpact = toi;
					}
				}
			}
		}
	}
	
}
