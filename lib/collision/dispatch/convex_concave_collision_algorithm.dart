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
import "package:bullet_physics/collision/broadphase/collision_algorithm_construction_info.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/collision_algorithm_create_func.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/convex_triangle_callback.dart";
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/narrowphase/convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/narrowphase/sub_simplex_convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/voronoi_simplex_solver.dart";
import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/sphere_shape.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/collision/shapes/triangle_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

class ConvexConcaveCollisionAlgorithm extends CollisionAlgorithm {
  static SwappedCCCAFunc SwappedCreateFunction() => SwappedCCCAFunc();
  static CCCAFunc CreateFunction() => CCCAFunc();
	bool _isSwapped = false;
	ConvexTriangleCallback? _btConvexTriangleCallback;
	
	void init(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1, bool isSwapped) {
		initCA(ci);
		_isSwapped = isSwapped;
		_btConvexTriangleCallback = ConvexTriangleCallback(dispatcher, body0, body1, isSwapped);
	}
	
	@override
	void destroy() {
		_btConvexTriangleCallback?.destroy();
	}

	@override
	void processCollision(CollisionObject? body0, CollisionObject? body1, DispatcherInfo? dispatchInfo, ManifoldResult? resultOut) {
		CollisionObject? convexBody = _isSwapped ? body1 : body0;
		CollisionObject? triBody = _isSwapped ? body0 : body1;

		if (triBody?.getCollisionShape()?.isConcave() ?? false) {
			CollisionObject? triOb = triBody;
			ConcaveShape? concaveShape = triOb?.getCollisionShape() as ConcaveShape?;

			if (convexBody?.getCollisionShape()?.isConvex() ?? false) {
				double collisionMarginTriangle = concaveShape!.getMargin();

				resultOut?.setPersistentManifold(_btConvexTriangleCallback?.manifoldPtr);
				_btConvexTriangleCallback?.setTimeStepAndCounters(collisionMarginTriangle, dispatchInfo, resultOut);
				_btConvexTriangleCallback?.manifoldPtr?.setBodies(convexBody, triBody);

				concaveShape.processAllTriangles(
						_btConvexTriangleCallback,
						_btConvexTriangleCallback?.getAabbMin(Vector3.zero()) ?? Vector3.zero(),
						_btConvexTriangleCallback?.getAabbMax(Vector3.zero()) ?? Vector3.zero()
        );

				resultOut?.refreshContactPoints();
			}
		}
	}

	@override
	double calculateTimeOfImpact(CollisionObject? body0, CollisionObject? body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
		Vector3 tmp = Vector3.zero();

		CollisionObject? convexbody = _isSwapped ? body1 : body0;
		CollisionObject? triBody = _isSwapped ? body0 : body1;

		// quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

		// only perform CCD above a certain threshold, this prevents blocking on the int run
		// because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
		tmp.sub2(convexbody?.getInterpolationWorldTransform(Transform()).origin, convexbody?.getWorldTransform(Transform()).origin);
		double squareMot0 = tmp.length2;
		if (squareMot0 < (convexbody?.getCcdSquareMotionThreshold() ?? 0)) {
			return 1;
		}

		Transform tmpTrans = Transform();
		Transform? triInv = triBody?.getWorldTransform(Transform());
		triInv?.inverse();

		Transform convexFromLocal = Transform();
		convexFromLocal.mul2(triInv!, convexbody!.getWorldTransform(tmpTrans));

		Transform convexToLocal = Transform();
		convexToLocal.mul2(triInv, convexbody.getInterpolationWorldTransform(tmpTrans));

		if (triBody?.getCollisionShape()?.isConcave() ?? false) {
			Vector3 rayAabbMin = Vector3.copy(convexFromLocal.origin);
			VectorUtil.setMin(rayAabbMin, convexToLocal.origin);

			Vector3 rayAabbMax = Vector3.copy(convexFromLocal.origin);
			VectorUtil.setMax(rayAabbMax, convexToLocal.origin);

			double ccdRadius0 = convexbody.getCcdSweptSphereRadius();

			tmp.setValues(ccdRadius0, ccdRadius0, ccdRadius0);
			rayAabbMin.sub(tmp);
			rayAabbMax.add(tmp);

			double curHitFraction = 1; // is this available?
			_LocalTriangleSphereCastCallback raycastCallback = _LocalTriangleSphereCastCallback(convexFromLocal, convexToLocal, convexbody.getCcdSweptSphereRadius(), curHitFraction);

			raycastCallback.hitFraction = convexbody.getHitFraction();

			CollisionObject? concavebody = triBody;

			ConcaveShape? triangleMesh = concavebody?.getCollisionShape() as ConcaveShape?;

			if (triangleMesh != null) {
				triangleMesh.processAllTriangles(raycastCallback, rayAabbMin, rayAabbMax);
			}

			if (raycastCallback.hitFraction < convexbody.getHitFraction()) {
				convexbody.setHitFraction(raycastCallback.hitFraction);
				return raycastCallback.hitFraction;
			}
		}

		return 1;
	}

	@override
	void getAllContactManifolds(ObjectArrayList<PersistentManifold> manifoldArray) {
		if (_btConvexTriangleCallback?.manifoldPtr != null) {
			manifoldArray.add(_btConvexTriangleCallback!.manifoldPtr!);
		}
	}
	
	void clearCache() {
		_btConvexTriangleCallback?.clearCache();
	}
}

////////////////////////////////////////////////////////////////////////////

class _LocalTriangleSphereCastCallback extends TriangleCallback {
  final Transform ccdSphereFromTrans = Transform();
  final Transform ccdSphereToTrans = Transform();
  final Transform meshTransform = Transform();

  double ccdSphereRadius = 0;
  double hitFraction = 0;
  
  final Transform _ident = Transform();
  
  _LocalTriangleSphereCastCallback(Transform from, Transform to, this.ccdSphereRadius, this.hitFraction) {
    ccdSphereFromTrans.copy(from);
    ccdSphereToTrans.copy(to);
    _ident.setIdentity();
  }
  @override
  void processTriangle(List<Vector3> triangle, int partId, int triangleIndex) {
    CastResult castResult = CastResult();
    castResult.fraction = hitFraction;
    SphereShape pointShape = SphereShape(ccdSphereRadius);
    TriangleShape triShape = TriangleShape(triangle[0], triangle[1], triangle[2]);
    VoronoiSimplexSolver simplexSolver = VoronoiSimplexSolver();
    SubSimplexConvexCast convexCaster = SubSimplexConvexCast(pointShape, triShape, simplexSolver);

    if (convexCaster.calcTimeOfImpact(ccdSphereFromTrans, ccdSphereToTrans, _ident, _ident, castResult)) {
      if (hitFraction > castResult.fraction) {
        hitFraction = castResult.fraction;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////
class CCCAFunc extends CollisionAlgorithmCreateFunc {
  @override
  CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
    ConvexConcaveCollisionAlgorithm algo = ConvexConcaveCollisionAlgorithm();
    algo.init(ci, body0, body1, false);
    return algo;
  }

  @override
  void releaseCollisionAlgorithm(CollisionAlgorithm algo) {}
}

class SwappedCCCAFunc extends CollisionAlgorithmCreateFunc {
  @override
  CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
    ConvexConcaveCollisionAlgorithm algo = ConvexConcaveCollisionAlgorithm();
    algo.init(ci, body0, body1, true);
    return algo;
  }

  @override
  void releaseCollisionAlgorithm(CollisionAlgorithm algo) {}
}