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

import "package:bullet_physics/collision/dispatch/collision_world.dart";
import "package:bullet_physics/dynamics/dynamics_world.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/dynamics/vehicle/vehicle_raycaster.dart";
import "package:bullet_physics/dynamics/vehicle/vehicle_raycaster_result.dart";
import 'package:vector_math/vector_math.dart';

class DefaultVehicleRaycaster extends VehicleRaycaster {
	late DynamicsWorld dynamicsWorld;

	DefaultVehicleRaycaster(DynamicsWorld world) {
		dynamicsWorld = world;
	}
  
  @override
	Object? castRay(Vector3 from, Vector3 to, VehicleRaycasterResult result) {
		//RayResultCallback& resultCallback;

		ClosestRayResultCallback rayCallback = ClosestRayResultCallback(from, to);

		dynamicsWorld.rayTest(from, to, rayCallback);

		if (rayCallback.hasHit()) {
			RigidBody? body = RigidBody.upcast(rayCallback.collisionObject);
			if (body != null && body.hasContactResponse()) {
				result.hitPointInWorld.setFrom(rayCallback.hitPointWorld);
				result.hitNormalInWorld.setFrom(rayCallback.hitNormalWorld);
				result.hitNormalInWorld.normalize();
				result.distFraction = rayCallback.closestHitFraction;
				return body;
			}
		}
		return null;
	}
}
