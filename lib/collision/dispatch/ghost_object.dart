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
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/collision_object_type.dart";
import "package:bullet_physics/collision/dispatch/collision_world.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';
import "package:bullet_physics/linearmath/aabb_util2.dart";

class GhostObject extends CollisionObject {
	ObjectArrayList<CollisionObject> overlappingObjects = ObjectArrayList();//List<CollisionObject>();

	GhostObject() {
		internalType = CollisionObjectType.ghostObject;
	}

	/**
	 * This method is mainly for expert/internal use only.
	 */
	void addOverlappingObjectInternal(BroadphaseProxy otherProxy, BroadphaseProxy thisProxy) {
		CollisionObject? otherObject = otherProxy.clientObject as CollisionObject?;
		assert(otherObject != null);
		
		// if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
		int index = overlappingObjects.indexOf(otherObject!);
		if (index == -1) {
			// not found
			overlappingObjects.add(otherObject);
		}
	}

	/**
	 * This method is mainly for expert/internal use only.
	 */
	void removeOverlappingObjectInternal(BroadphaseProxy? otherProxy, Dispatcher? dispatcher, BroadphaseProxy? thisProxy) {
		CollisionObject? otherObject = otherProxy?.clientObject as CollisionObject?;
		assert(otherObject != null);
		
		int index = overlappingObjects.indexOf(otherObject!);
		if (index != -1) {
			overlappingObjects.set(index, overlappingObjects.getQuick(overlappingObjects.size-1)!);
			overlappingObjects.removeAt(overlappingObjects.size-1);
		}
	}

	void convexSweepTest(ConvexShape castShape, Transform convexFromWorld, Transform convexToWorld, ConvexResultCallback resultCallback, double allowedCcdPenetration) {
		Transform convexFromTrans = Transform();
		Transform convexToTrans = Transform();

		convexFromTrans.copy(convexFromWorld);
		convexToTrans.copy(convexToWorld);

		Vector3 castShapeAabbMin = Vector3.zero();
		Vector3 castShapeAabbMax = Vector3.zero();

		// compute AABB that encompasses angular movement
		{
			Vector3 linVel = Vector3.zero();
			Vector3 angVel = Vector3.zero();
			TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1, linVel, angVel);
			Transform R = Transform();
			R.setIdentity();
			R.setRotation(convexFromTrans.getRotation(Quaternion(0,0,0,0)));
			castShape.calculateTemporalAabb(R, linVel, angVel, 1, castShapeAabbMin, castShapeAabbMax);
		}

		Transform tmpTrans = Transform();

		// go over all objects, and if the ray intersects their aabb + cast shape aabb,
		// do a ray-shape query using convexCaster (CCD)
		for (int i=0; i<overlappingObjects.size; i++) {
			CollisionObject? collisionObject = overlappingObjects.getQuick(i);

			// only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject?.getBroadphaseHandle())) {
				//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
				Vector3 collisionObjectAabbMin = Vector3.zero();
				Vector3 collisionObjectAabbMax = Vector3.zero();
				collisionObject?.getCollisionShape()?.getAabb(collisionObject.getWorldTransform(tmpTrans), collisionObjectAabbMin, collisionObjectAabbMax);
				AabbUtil2.aabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
				double hitLambda = 1; // could use resultCallback.closestHitFraction, but needs testing
				Vector3 hitNormal = Vector3.zero();
				if (AabbUtil2.rayAabb(convexFromWorld.origin, convexToWorld.origin, collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal)) {
					CollisionWorld.objectQuerySingle(
            castShape, 
            convexFromTrans, 
            convexToTrans,
            collisionObject,
            collisionObject?.getCollisionShape(),
            collisionObject?.getWorldTransform(tmpTrans),
            resultCallback,
            allowedCcdPenetration
          );
				}
			}
		}
	}

	void rayTest(Vector3 rayFromWorld, Vector3 rayToWorld, RayResultCallback resultCallback) {
		Transform rayFromTrans = Transform();
		rayFromTrans.setIdentity();
		rayFromTrans.origin.setFrom(rayFromWorld);
		Transform rayToTrans = Transform();
		rayToTrans.setIdentity();
		rayToTrans.origin.setFrom(rayToWorld);

		Transform tmpTrans = Transform();

		for (int i=0; i<overlappingObjects.size; i++) {
			CollisionObject? collisionObject = overlappingObjects.getQuick(i);
			
			// only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject?.getBroadphaseHandle())) {
				CollisionWorld.rayTestSingle(
          rayFromTrans, 
          rayToTrans,
          collisionObject,
          collisionObject?.getCollisionShape(),
          collisionObject?.getWorldTransform(tmpTrans),
          resultCallback
        );
			}
		}
	}

	int getNumOverlappingObjects() {
		return overlappingObjects.size;
	}

	CollisionObject? getOverlappingObject(int index) {
		return overlappingObjects.getQuick(index);
	}

	ObjectArrayList<CollisionObject> getOverlappingPairs() {
		return overlappingObjects;
	}

	//
	// internal cast
	//

	static GhostObject? upcast(CollisionObject? colObj) {
		if (colObj?.getInternalType() == CollisionObjectType.ghostObject) {
			return colObj as GhostObject;
		}
		
		return null;
	}
}
