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

import "package:bullet_physics/collision/broadphase/broadphase_interface.dart";
import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/broadphase/broadphase_proxy.dart";
import "package:bullet_physics/collision/broadphase/collision_filter_groups.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart";
import "package:bullet_physics/collision/dispatch/collision_configuration.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/narrowphase/convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/gjk_convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/sub_simplex_convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/triangle_convexcast_callback.dart";
import "package:bullet_physics/collision/narrowphase/triangle_raycast_callback.dart";
import "package:bullet_physics/collision/narrowphase/voronoi_simplex_solver.dart";
import "package:bullet_physics/collision/shapes/bvh_triangle_mesh_shape.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/compound_shape.dart";
import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/core/bullet_stats.dart";
import "package:bullet_physics/linearmath/i_debug_draw.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';
import "package:bullet_physics/collision/shapes/sphere_shape.dart";
import "package:bullet_physics/linearmath/aabb_util2.dart";

class CollisionWorld {
	//final BulletStack stack = BulletStack.get();
	ObjectArrayList<CollisionObject> collisionObjects = ObjectArrayList();
	Dispatcher? dispatcher1;
	DispatcherInfo dispatchInfo = DispatcherInfo();
	BroadphaseInterface broadphasePairCache;
	IDebugDraw? debugDrawer;
	
	/**
	 * This constructor doesn't own the dispatcher and paircache/broadphase.
	 */
	CollisionWorld(this.dispatcher1,this.broadphasePairCache, CollisionConfiguration collisionConfiguration);
	
	void destroy() {
		// clean up remaining objects
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? collisionObject = collisionObjects.getQuick(i);

			BroadphaseProxy? bp = collisionObject?.getBroadphaseHandle();
			if (bp != null) {
				getBroadphase().getOverlappingPairCache()?.cleanProxyFromPairs(bp, dispatcher1);
				getBroadphase().destroyProxy(bp, dispatcher1);
			}
		}
	}

	void addCollisionObject(CollisionObject collisionObject, [int collisionFilterGroup = CollisionFilterGroups.defaultFilter, int collisionFilterMask = CollisionFilterGroups.allFilter]) {
		// check that the object isn't already added
		assert (!collisionObjects.contains(collisionObject));

		collisionObjects.add(collisionObject);

		// calculate AABB
		Transform trans = collisionObject.getWorldTransform(Transform());

		Vector3 minAabb = Vector3.zero();
		Vector3 maxAabb = Vector3.zero();
		collisionObject.getCollisionShape()?.getAabb(trans, minAabb, maxAabb);

		BroadphaseNativeType type = collisionObject.getCollisionShape()?.getShapeType() ?? BroadphaseNativeType.values[0];
		collisionObject.setBroadphaseHandle(
      getBroadphase().createProxy(
				minAabb,
				maxAabb,
				type,
				collisionObject,
				collisionFilterGroup,
				collisionFilterMask,
				dispatcher1, 
        null
      )
    );
	}

	void performDiscreteCollisionDetection() {
		BulletStats.pushProfile("performDiscreteCollisionDetection");
		try {
			updateAabbs();
			BulletStats.pushProfile("calculateOverlappingPairs");
			try {
				broadphasePairCache.calculateOverlappingPairs(dispatcher1);
			}
			finally {
				BulletStats.popProfile();
			}
      
			Dispatcher? dispatcher = getDispatcher();
      BulletStats.pushProfile("dispatchAllCollisionPairs");
      try {
        dispatcher?.dispatchAllCollisionPairs(broadphasePairCache.getOverlappingPairCache(), dispatchInfo, dispatcher1);
      }
      finally {
        BulletStats.popProfile();
      }
			
		}
		finally {
			BulletStats.popProfile();
		}
	}
	
	void removeCollisionObject(CollisionObject collisionObject) {
    BroadphaseProxy? bp = collisionObject.getBroadphaseHandle();
    if (bp != null) {
      getBroadphase().getOverlappingPairCache()?.cleanProxyFromPairs(bp, dispatcher1);
      getBroadphase().destroyProxy(bp, dispatcher1);
      collisionObject.setBroadphaseHandle(null);
    }
		//swapremove
		collisionObjects.remove(collisionObject);
	}

	void setBroadphase(BroadphaseInterface pairCache) {
		broadphasePairCache = pairCache;
	}
	
	BroadphaseInterface getBroadphase() {
		return broadphasePairCache;
	}
	
	OverlappingPairCache? getPairCache() {
		return broadphasePairCache.getOverlappingPairCache();
	}

	Dispatcher? getDispatcher() {
		return dispatcher1;
	}

	DispatcherInfo getDispatchInfo() {
		return dispatchInfo;
	}
	
	static bool _updateAabbsReportMe = true;

	// JAVA NOTE: ported from 2.74, missing contact threshold stuff
	void updateSingleAabb(CollisionObject colObj) {
		Vector3 minAabb = Vector3.zero(), maxAabb = Vector3.zero();
		Vector3 tmp = Vector3.zero();
		Transform tmpTrans = Transform();

		colObj.getCollisionShape()?.getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
		// need to increase the aabb for contact thresholds
		Vector3 contactThreshold = Vector3.zero();
		contactThreshold.setValues(BulletGlobals.getContactBreakingThreshold(), BulletGlobals.getContactBreakingThreshold(), BulletGlobals.getContactBreakingThreshold());
		minAabb.sub(contactThreshold);
		maxAabb.add(contactThreshold);

		BroadphaseInterface bp = broadphasePairCache;

		// moving objects should be moderately sized, probably something wrong if not
		tmp.sub2(maxAabb,minAabb); // TODO: optimize
		if (colObj.isStaticObject() || (tmp.length2 < 1e12)) {
			bp.setAabb(colObj.getBroadphaseHandle(), minAabb, maxAabb, dispatcher1);
		}
		else {
			// something went wrong, investigate
			// this assert is unwanted in 3D modelers (danger of loosing work)
			colObj.setActivationState(CollisionObject.disableSimulation);

			if (_updateAabbsReportMe && debugDrawer != null) {
				_updateAabbsReportMe = false;
				debugDrawer?.reportErrorWarning("Overflow in AABB, object removed from simulation");
				debugDrawer?.reportErrorWarning("If you can reproduce this, please email bugs@continuousphysics.com\n");
				debugDrawer?.reportErrorWarning("Please include above information, your Platform, version of OS.\n");
				debugDrawer?.reportErrorWarning("Thanks.\n");
			}
		}
	}

	void updateAabbs() {
		BulletStats.pushProfile("updateAabbs");
		try {
			for (int i=0; i<collisionObjects.size; i++) {
				CollisionObject? colObj = collisionObjects.getQuick(i);

				// only update aabb of active objects
				if (colObj?.isActive() ?? false) {
					updateSingleAabb(colObj!);
				}
			}
		}
		finally {
			BulletStats.popProfile();
		}
	}

	IDebugDraw? getDebugDrawer() {
		return debugDrawer;
	}

	void setDebugDrawer(IDebugDraw debugDrawer) {
		this.debugDrawer = debugDrawer;
	}
	
	int getNumCollisionObjects() {
		return collisionObjects.size;
	}

	// TODO
	static void rayTestSingle(
    Transform rayFromTrans, 
    Transform rayToTrans,
    CollisionObject? collisionObject,
    CollisionShape? collisionShape,
    Transform? colObjWorldTransform,
    RayResultCallback resultCallback
  ) {
		SphereShape pointShape = SphereShape(0);
		pointShape.setMargin(0);
		ConvexShape castShape = pointShape;

		if (collisionShape!.isConvex()) {
			CastResult castResult = CastResult();
			castResult.fraction = resultCallback.closestHitFraction;

			ConvexShape convexShape = collisionShape as ConvexShape;
			VoronoiSimplexSolver simplexSolver = VoronoiSimplexSolver();
			SubSimplexConvexCast convexCaster = SubSimplexConvexCast(castShape, convexShape, simplexSolver);

			if (convexCaster.calcTimeOfImpact(rayFromTrans, rayToTrans, colObjWorldTransform!, colObjWorldTransform, castResult)) {
				//add hit
				if (castResult.normal.length2 > 0.0001) {
					if (castResult.fraction < resultCallback.closestHitFraction) {
						rayFromTrans.basis.transform(castResult.normal);
						castResult.normal.normalize();

						LocalRayResult localRayResult = LocalRayResult(
								collisionObject,
								null,
								castResult.normal,
								castResult.fraction);

						bool normalInWorldSpace = true;
						resultCallback.addSingleResult(localRayResult, normalInWorldSpace);
					}
				}
			}
		}
		else {
			if (collisionShape.isConcave()) {
				if (collisionShape.getShapeType() == BroadphaseNativeType.triangleMeshShapeProxytype) {
					// optimized version for BvhTriangleMeshShape
					BvhTriangleMeshShape triangleMesh = collisionShape as BvhTriangleMeshShape;
					Transform worldTocollisionObject = Transform();
					worldTocollisionObject.inverse(colObjWorldTransform);
					Vector3 rayFromLocal = Vector3.copy(rayFromTrans.origin);
					worldTocollisionObject.transform(rayFromLocal);
					Vector3 rayToLocal = Vector3.copy(rayToTrans.origin);
					worldTocollisionObject.transform(rayToLocal);

					_BridgeTriangleRaycastCallback rcb = _BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal, resultCallback, collisionObject, triangleMesh);
					rcb.hitFraction = resultCallback.closestHitFraction;
					triangleMesh.performRaycast(rcb, rayFromLocal, rayToLocal);
				}
				else {
					ConcaveShape triangleMesh = collisionShape as ConcaveShape;

					Transform worldTocollisionObject = Transform();
					worldTocollisionObject.inverse(colObjWorldTransform);

					Vector3 rayFromLocal = Vector3.copy(rayFromTrans.origin);
					worldTocollisionObject.transform(rayFromLocal);
					Vector3 rayToLocal = Vector3.copy(rayToTrans.origin);
					worldTocollisionObject.transform(rayToLocal);

					_BridgeTriangleRaycastCallback rcb = _BridgeTriangleRaycastCallback(rayFromLocal, rayToLocal, resultCallback, collisionObject, triangleMesh);
					rcb.hitFraction = resultCallback.closestHitFraction;

					Vector3 rayAabbMinLocal = Vector3.copy(rayFromLocal);
					VectorUtil.setMin(rayAabbMinLocal, rayToLocal);
					Vector3 rayAabbMaxLocal = Vector3.copy(rayFromLocal);
					VectorUtil.setMax(rayAabbMaxLocal, rayToLocal);

					triangleMesh.processAllTriangles(rcb, rayAabbMinLocal, rayAabbMaxLocal);
				}
			}
			else {
				// todo: use AABB tree or other BVH acceleration structure!
				if (collisionShape.isCompound()) {
					CompoundShape compoundShape = collisionShape as CompoundShape;
					Transform childTrans = Transform();
					for (int i = 0; i < compoundShape.getNumChildShapes(); i++) {
						compoundShape.getChildTransform(i, childTrans);
						CollisionShape? childCollisionShape = compoundShape.getChildShape(i);
						Transform childWorldTrans = Transform.formTransfrom(colObjWorldTransform);
						childWorldTrans.mul(childTrans);
						// replace collision shape so that callback can determine the triangle
						CollisionShape? saveCollisionShape = collisionObject?.getCollisionShape();
						collisionObject?.internalSetTemporaryCollisionShape(childCollisionShape);
						rayTestSingle(rayFromTrans, rayToTrans,
								collisionObject,
								childCollisionShape,
								childWorldTrans,
								resultCallback);
						// restore
						collisionObject?.internalSetTemporaryCollisionShape(saveCollisionShape);
					}
				}
			}
		}
	}

	/**
	 * objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
	 */
	static void objectQuerySingle(
    ConvexShape castShape, 
    Transform convexFromTrans, 
    Transform convexToTrans, 
    CollisionObject? collisionObject, 
    CollisionShape? collisionShape, 
    Transform? colObjWorldTransform, 
    ConvexResultCallback resultCallback, 
    double allowedPenetration
  ) {
		if (collisionShape?.isConvex() ?? false) {
			CastResult castResult = CastResult();
			castResult.allowedPenetration = allowedPenetration;
			castResult.fraction = 1;

			ConvexShape convexShape = collisionShape as ConvexShape;
			VoronoiSimplexSolver simplexSolver = VoronoiSimplexSolver();
			GjkConvexCast convexCaster2 = GjkConvexCast(castShape, convexShape, simplexSolver);
			ConvexCast castPtr = convexCaster2 as ConvexCast;

			if (castPtr.calcTimeOfImpact(
        convexFromTrans, 
        convexToTrans,
        colObjWorldTransform, 
        colObjWorldTransform, 
        castResult
        )
      ) {
				// add hit
				if (castResult.normal.length2 > 0.0001) {
					if (castResult.fraction < resultCallback.closestHitFraction) {
						castResult.normal.normalize();
						LocalConvexResult localConvexResult = LocalConvexResult(collisionObject, null, castResult.normal, castResult.hitPoint, castResult.fraction);

						bool normalInWorldSpace = true;
						resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
					}
				}
			}
		}
		else {
			if (collisionShape?.isConcave() ?? false) {
				if (collisionShape?.getShapeType() == BroadphaseNativeType.triangleMeshShapeProxytype) {
					BvhTriangleMeshShape triangleMesh = collisionShape as BvhTriangleMeshShape;
					Transform worldTocollisionObject = Transform();
					worldTocollisionObject.inverse(colObjWorldTransform);

					Vector3 convexFromLocal = Vector3.zero();
					convexFromLocal.setFrom(convexFromTrans.origin);
					worldTocollisionObject.transform(convexFromLocal);

					Vector3 convexToLocal = Vector3.zero();
					convexToLocal.setFrom(convexToTrans.origin);
					worldTocollisionObject.transform(convexToLocal);

					// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
					Transform rotationXform = Transform();
					Matrix3 tmpMat = Matrix3.zero();
					tmpMat.mul2(worldTocollisionObject.basis, convexToTrans.basis);
					rotationXform.setFromMatrix3(tmpMat);

					_BridgeTriangleConvexcastCallback tccb = _BridgeTriangleConvexcastCallback(castShape, convexFromTrans, convexToTrans, resultCallback, collisionObject, triangleMesh, colObjWorldTransform);
					tccb.hitFraction = resultCallback.closestHitFraction;
					tccb.normalInWorldSpace = true;
					
					Vector3 boxMinLocal = Vector3.zero();
					Vector3 boxMaxLocal = Vector3.zero();
					castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);
					triangleMesh.performConvexcast(tccb, convexFromLocal, convexToLocal, boxMinLocal, boxMaxLocal);
				}
				else {
					ConcaveShape triangleMesh = collisionShape as ConcaveShape;
					Transform worldTocollisionObject = Transform();
					worldTocollisionObject.inverse(colObjWorldTransform);

					Vector3 convexFromLocal = Vector3.zero();
					convexFromLocal.setFrom(convexFromTrans.origin);
					worldTocollisionObject.transform(convexFromLocal);

					Vector3 convexToLocal = Vector3.zero();
					convexToLocal.setFrom(convexToTrans.origin);
					worldTocollisionObject.transform(convexToLocal);

					// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
					Transform rotationXform = Transform();
					Matrix3 tmpMat = Matrix3.zero();
					tmpMat.mul2(worldTocollisionObject.basis, convexToTrans.basis);
					rotationXform.setFromMatrix3(tmpMat);

					_BridgeTriangleConvexcastCallback tccb = _BridgeTriangleConvexcastCallback(castShape, convexFromTrans, convexToTrans, resultCallback, collisionObject, triangleMesh, colObjWorldTransform);
					tccb.hitFraction = resultCallback.closestHitFraction;
					tccb.normalInWorldSpace = false;
					Vector3 boxMinLocal = Vector3.zero();
					Vector3 boxMaxLocal = Vector3.zero();
					castShape.getAabb(rotationXform, boxMinLocal, boxMaxLocal);

					Vector3 rayAabbMinLocal = Vector3.copy(convexFromLocal);
					VectorUtil.setMin(rayAabbMinLocal, convexToLocal);
					Vector3 rayAabbMaxLocal = Vector3.copy(convexFromLocal);
					VectorUtil.setMax(rayAabbMaxLocal, convexToLocal);
					rayAabbMinLocal.add(boxMinLocal);
					rayAabbMaxLocal.add(boxMaxLocal);
					triangleMesh.processAllTriangles(tccb, rayAabbMinLocal, rayAabbMaxLocal);
				}
			}
			else {
				// todo: use AABB tree or other BVH acceleration structure!
				if (collisionShape?.isCompound() ?? false) {
					CompoundShape compoundShape = collisionShape as CompoundShape;
					for (int i = 0; i < compoundShape.getNumChildShapes(); i++) {
						Transform childTrans = compoundShape.getChildTransform(i, Transform());
						CollisionShape? childCollisionShape = compoundShape.getChildShape(i);
						Transform childWorldTrans = Transform();
						childWorldTrans.mul2(colObjWorldTransform!, childTrans);
						// replace collision shape so that callback can determine the triangle
						CollisionShape? saveCollisionShape = collisionObject?.getCollisionShape();
						collisionObject?.internalSetTemporaryCollisionShape(childCollisionShape);
						objectQuerySingle(
              castShape, 
              convexFromTrans, 
              convexToTrans,
              collisionObject,
              childCollisionShape,
              childWorldTrans,
              resultCallback, 
              allowedPenetration
            );
						// restore
						collisionObject?.internalSetTemporaryCollisionShape(saveCollisionShape);
					}
				}
			}
		}
	}

	/**
	 * rayTest performs a raycast on all objects in the CollisionWorld, and calls the resultCallback.
	 * This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
	 */
	void rayTest(Vector3 rayFromWorld, Vector3 rayToWorld, RayResultCallback resultCallback) {
		Transform rayFromTrans = Transform(), rayToTrans = Transform();
		rayFromTrans.setIdentity();
		rayFromTrans.origin.setFrom(rayFromWorld);
		rayToTrans.setIdentity();

		rayToTrans.origin.setFrom(rayToWorld);

		// go over all objects, and if the ray intersects their aabb, do a ray-shape query using convexCaster (CCD)
		Vector3 collisionObjectAabbMin = Vector3.zero(), collisionObjectAabbMax = Vector3.zero();
		double hitLambda = 0;

		Transform tmpTrans = Transform();
		
		for (int i = 0; i < collisionObjects.size; i++) {
			// terminate further ray tests, once the closestHitFraction reached zero
			if (resultCallback.closestHitFraction == 0) {
				break;
			}

			CollisionObject? collisionObject = collisionObjects.getQuick(i);
			// only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject?.getBroadphaseHandle())) {
				collisionObject?.getCollisionShape()?.getAabb(collisionObject.getWorldTransform(tmpTrans), collisionObjectAabbMin, collisionObjectAabbMax);

				hitLambda = resultCallback.closestHitFraction;
				Vector3 hitNormal = Vector3.zero();
				if (AabbUtil2.rayAabb(rayFromWorld, rayToWorld, collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal)) {
					rayTestSingle(
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
	}

	/**
	 * convexTest performs a swept convex cast on all objects in the {@link CollisionWorld}, and calls the resultCallback
	 * This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
	 */
	void convexSweepTest(ConvexShape castShape, Transform convexFromWorld, Transform convexToWorld, ConvexResultCallback resultCallback) {
		Transform convexFromTrans = Transform();
		Transform convexToTrans = Transform();

		convexFromTrans.copy(convexFromWorld);
		convexToTrans.copy(convexToWorld);

		Vector3 castShapeAabbMin = Vector3.zero();
		Vector3 castShapeAabbMax = Vector3.zero();

		// Compute AABB that encompasses angular movement
    Vector3 linVel = Vector3.zero();
    Vector3 angVel = Vector3.zero();
    TransformUtil.calculateVelocity(convexFromTrans, convexToTrans, 1, linVel, angVel);
    Transform R = Transform();
    R.setIdentity();
    R.setRotation(convexFromTrans.getRotation(Quaternion(0,0,0,1)));
    castShape.calculateTemporalAabb(R, linVel, angVel, 1, castShapeAabbMin, castShapeAabbMax);
		
		Transform tmpTrans = Transform();
		Vector3 collisionObjectAabbMin = Vector3.zero();
		Vector3 collisionObjectAabbMax = Vector3.zero();
		double hitLambda = 0;

		// go over all objects, and if the ray intersects their aabb + cast shape aabb,
		// do a ray-shape query using convexCaster (CCD)
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? collisionObject = collisionObjects.getQuick(i);

			// only perform raycast if filterMask matches
			if (resultCallback.needsCollision(collisionObject?.getBroadphaseHandle())) {
				//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
				collisionObject?.getWorldTransform(tmpTrans);
				collisionObject?.getCollisionShape()?.getAabb(tmpTrans, collisionObjectAabbMin, collisionObjectAabbMax);
				AabbUtil2.aabbExpand(collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
				hitLambda = 1; // could use resultCallback.closestHitFraction, but needs testing
				Vector3 hitNormal = Vector3.zero();
				if (AabbUtil2.rayAabb(convexFromWorld.origin, convexToWorld.origin, collisionObjectAabbMin, collisionObjectAabbMax, hitLambda, hitNormal)) {
					objectQuerySingle(
            castShape, 
            convexFromTrans, 
            convexToTrans,
            collisionObject,
            collisionObject?.getCollisionShape(),
            tmpTrans,
            resultCallback,
            getDispatchInfo().allowedCcdPenetration
          );
				}
			}
		}
	}

	ObjectArrayList<CollisionObject> getCollisionObjectArray() {
		return collisionObjects;
	}	
}

////////////////////////////////////////////////////////////////////////////

/**
 * LocalShapeInfo gives extra information for complex shapes.
 * Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart.
 */
class LocalShapeInfo {
  int shapePart = 0;
  int triangleIndex = 0;
}

class LocalRayResult {
  CollisionObject? collisionObject;
  LocalShapeInfo? localShapeInfo;
  final Vector3 hitNormalLocal = Vector3.zero();
  double hitFraction = 0;

  LocalRayResult(this.collisionObject, this.localShapeInfo, Vector3 hitNormalLocal, this.hitFraction) {
    this.hitNormalLocal.setFrom(hitNormalLocal);
  }
}

/**
 * RayResultCallback is used to report raycast results.
 */
abstract class RayResultCallback {
  double closestHitFraction = 1;
  CollisionObject? collisionObject;
  int collisionFilterGroup = CollisionFilterGroups.defaultFilter;
  int collisionFilterMask = CollisionFilterGroups.allFilter;
  
  bool hasHit() {
    return (collisionObject != null);
  }

  bool needsCollision(BroadphaseProxy? proxy0) {
    //if(proxy0 == null) return false;
    bool collides = ((proxy0!.collisionFilterGroup & collisionFilterMask) & 0xFFFF) != 0;
    collides = collides && ((collisionFilterGroup & proxy0.collisionFilterMask) & 0xFFFF) != 0;
    return collides;
  }
  
  double addSingleResult(LocalRayResult rayResult, bool normalInWorldSpace);
}

class ClosestRayResultCallback extends RayResultCallback {
  final Vector3 rayFromWorld = Vector3.zero(); //used to calculate hitPointWorld from hitFraction
  final Vector3 rayToWorld = Vector3.zero();

  final Vector3 hitNormalWorld = Vector3.zero();
  final Vector3 hitPointWorld = Vector3.zero();
  
  ClosestRayResultCallback(Vector3 rayFromWorld, Vector3 rayToWorld) {
    this.rayFromWorld.setFrom(rayFromWorld);
    this.rayToWorld.setFrom(rayToWorld);
  }
  
  @override
  double addSingleResult(LocalRayResult rayResult, bool normalInWorldSpace) {
    // caller already does the filter on the closestHitFraction
    assert (rayResult.hitFraction <= closestHitFraction);

    closestHitFraction = rayResult.hitFraction;
    collisionObject = rayResult.collisionObject;
    if (normalInWorldSpace) {
      hitNormalWorld.setFrom(rayResult.hitNormalLocal);
    }
    else {
      // need to transform normal into worldspace
      hitNormalWorld.setFrom(rayResult.hitNormalLocal);
      collisionObject?.getWorldTransform(Transform()).basis.transform(hitNormalWorld);
    }

    VectorUtil.setInterpolate3(hitPointWorld, rayFromWorld, rayToWorld, rayResult.hitFraction);
    return rayResult.hitFraction;
  }
}

class LocalConvexResult {
  CollisionObject? hitCollisionObject;
  LocalShapeInfo? localShapeInfo;
  final Vector3 hitNormalLocal = Vector3.zero();
  final Vector3 hitPointLocal = Vector3.zero();
  double hitFraction;

  LocalConvexResult(this.hitCollisionObject, this.localShapeInfo, Vector3 hitNormalLocal, Vector3 hitPointLocal, this.hitFraction) {
    this.hitNormalLocal.setFrom(hitNormalLocal);
    this.hitPointLocal.setFrom(hitPointLocal);
  }
}

abstract class ConvexResultCallback {
  double closestHitFraction = 1;
  int collisionFilterGroup = CollisionFilterGroups.defaultFilter;
  int collisionFilterMask = CollisionFilterGroups.allFilter;
  
  bool hasHit() {
    return (closestHitFraction < 1);
  }
  
  bool needsCollision(BroadphaseProxy? proxy0) {
    if(proxy0 == null) return false;
    bool collides = ((proxy0.collisionFilterGroup & collisionFilterMask) & 0xFFFF) != 0;
    collides = collides && ((collisionFilterGroup & proxy0.collisionFilterMask) & 0xFFFF) != 0;
    return collides;
  }
  
  double addSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace);
}

class ClosestConvexResultCallback extends ConvexResultCallback {
  final Vector3 convexFromWorld = Vector3.zero(); // used to calculate hitPointWorld from hitFraction
  final Vector3 convexToWorld = Vector3.zero();
  final Vector3 hitNormalWorld = Vector3.zero();
  final Vector3 hitPointWorld = Vector3.zero();
  CollisionObject? hitCollisionObject;

  ClosestConvexResultCallback(Vector3 convexFromWorld, Vector3 convexToWorld) {
    this.convexFromWorld.setFrom(convexFromWorld);
    this.convexToWorld.setFrom(convexToWorld);
  }

  @override
  double addSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace) {
    // caller already does the filter on the m_closestHitFraction
    assert (convexResult.hitFraction <= closestHitFraction);

    closestHitFraction = convexResult.hitFraction;
    hitCollisionObject = convexResult.hitCollisionObject;
    if (normalInWorldSpace) {
      hitNormalWorld.setFrom(convexResult.hitNormalLocal);
      if (hitNormalWorld.length > 2) {
        print("CollisionWorld.addSingleResult world $hitNormalWorld");
      }
    }
    else {
      // need to transform normal into worldspace
      hitNormalWorld.setFrom(convexResult.hitNormalLocal);
      hitCollisionObject?.getWorldTransform(Transform()).basis.transform(hitNormalWorld);
      if (hitNormalWorld.length > 2) {
        print("CollisionWorld.addSingleResult world $hitNormalWorld");
      }
    }

    hitPointWorld.setFrom(convexResult.hitPointLocal);
    return convexResult.hitFraction;
  }
}

class _BridgeTriangleRaycastCallback extends TriangleRaycastCallback {
  RayResultCallback resultCallback;
  CollisionObject? collisionObject;
  ConcaveShape triangleMesh;

  _BridgeTriangleRaycastCallback(super.from, super.to, this.resultCallback, this.collisionObject, this.triangleMesh);

  @override
  double reportHit(Vector3 hitNormalLocal, double hitFraction, int partId, int triangleIndex) {
    LocalShapeInfo shapeInfo = LocalShapeInfo();
    shapeInfo.shapePart = partId;
    shapeInfo.triangleIndex = triangleIndex;

    LocalRayResult rayResult = LocalRayResult(collisionObject, shapeInfo, hitNormalLocal, hitFraction);

    bool normalInWorldSpace = false;
    return resultCallback.addSingleResult(rayResult, normalInWorldSpace);
  }
}

class _BridgeTriangleConvexcastCallback extends TriangleConvexcastCallback {
  ConvexResultCallback resultCallback;
  CollisionObject? collisionObject;
  ConcaveShape triangleMesh;
  bool normalInWorldSpace = false;

  _BridgeTriangleConvexcastCallback(
    ConvexShape castShape, 
    Transform from, 
    Transform to, 
    this.resultCallback, 
    this.collisionObject, 
    this.triangleMesh, 
    Transform? triangleToWorld
  ):super(castShape, from, to, triangleToWorld, triangleMesh.getMargin());

  @override
  double reportHit(Vector3 hitNormalLocal, Vector3 hitPointLocal, double hitFraction, int partId, int triangleIndex) {
    LocalShapeInfo shapeInfo = LocalShapeInfo();
    shapeInfo.shapePart = partId;
    shapeInfo.triangleIndex = triangleIndex;
    if (hitFraction <= resultCallback.closestHitFraction) {
      LocalConvexResult convexResult = LocalConvexResult(collisionObject, shapeInfo, hitNormalLocal, hitPointLocal, hitFraction);
      return resultCallback.addSingleResult(convexResult, normalInWorldSpace);
    }
    return hitFraction;
  }
}