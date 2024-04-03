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

import "package:bullet_physics/collision/broadphase/broadphase_pair.dart";
import "package:bullet_physics/collision/broadphase/broadphase_proxy.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_callback.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/ghost_object.dart";

class GhostPairCallback extends OverlappingPairCallback {
  @override
	BroadphasePair? addOverlappingPair(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1) {
		CollisionObject? colObj0 = proxy0?.clientObject as CollisionObject?;
		CollisionObject? colObj1 = proxy1?.clientObject as CollisionObject?;
		GhostObject? ghost0 = GhostObject.upcast(colObj0);
		GhostObject? ghost1 = GhostObject.upcast(colObj1);

		ghost0?.addOverlappingObjectInternal(proxy1!, proxy0!);
		ghost1?.addOverlappingObjectInternal(proxy0!, proxy1!);
	
		return null;
	}
  @override
	Object? removeOverlappingPair(BroadphaseProxy? proxy0, BroadphaseProxy? proxy1, Dispatcher? dispatcher) {
		CollisionObject? colObj0 = proxy0?.clientObject as CollisionObject?;
		CollisionObject? colObj1 = proxy1?.clientObject as CollisionObject?;
		GhostObject? ghost0 = GhostObject.upcast(colObj0);
		GhostObject? ghost1 = GhostObject.upcast(colObj1);

    ghost0?.removeOverlappingObjectInternal(proxy1, dispatcher, proxy0);
    ghost1?.removeOverlappingObjectInternal(proxy0, dispatcher, proxy1);

		return null;
	}
  @override
	void removeOverlappingPairsContainingProxy(BroadphaseProxy proxy0, Dispatcher? dispatcher) {
		assert (false);
	}
	
}
