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
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/collision/shapes/static_plane_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

class ConvexPlaneCollisionAlgorithm extends CollisionAlgorithm {
  static CPCAFunc CreateFunction() => CPCAFunc();
	bool _ownManifold = false;
	PersistentManifold? _manifoldPtr;
	bool _isSwapped = false;
	
	void init(PersistentManifold? mf, CollisionAlgorithmConstructionInfo ci, CollisionObject? col0, CollisionObject? col1, bool isSwapped) {
		initCA(ci);
		_ownManifold = false;
		_manifoldPtr = mf;
		_isSwapped = isSwapped;

		CollisionObject? convexObj = _isSwapped ? col1 : col0;
		CollisionObject? planeObj = _isSwapped ? col0 : col1;

		if (_manifoldPtr == null && (dispatcher?.needsCollision(convexObj, planeObj) ?? false)) {
			_manifoldPtr = dispatcher?.getNewManifold(convexObj, planeObj);
			_ownManifold = true;
		}
	}
	
	@override
	void destroy() {
		if (_ownManifold) {
			if (_manifoldPtr != null) {
				dispatcher?.releaseManifold(_manifoldPtr);
			}
			_manifoldPtr = null;
		}
	}
	
	@override
	void processCollision(CollisionObject? body0, CollisionObject? body1, DispatcherInfo? dispatchInfo, ManifoldResult? resultOut) {
		if (_manifoldPtr == null) {
			return;
		}
		
		Transform tmpTrans = Transform();

		CollisionObject? convexObj = _isSwapped ? body1 : body0;
		CollisionObject? planeObj = _isSwapped ? body0 : body1;

		ConvexShape? convexShape = convexObj?.getCollisionShape() as ConvexShape?;
		StaticPlaneShape? planeShape = planeObj?.getCollisionShape() as StaticPlaneShape?;

		bool hasCollision = false;
		Vector3 planeNormal = planeShape?.getPlaneNormal(Vector3.zero()) ?? Vector3.zero();
		double planeConstant = planeShape?.getPlaneConstant() ?? 0;

		Transform planeInConvex = Transform();
		convexObj?.getWorldTransform(planeInConvex);
		planeInConvex.inverse();
		planeInConvex.mul(planeObj?.getWorldTransform(tmpTrans));

		Transform convexInPlaneTrans = Transform();
		convexInPlaneTrans.inverse(planeObj?.getWorldTransform(tmpTrans));
		convexInPlaneTrans.mul(convexObj?.getWorldTransform(tmpTrans));

		Vector3 tmp = Vector3.zero();
		tmp.negateFrom(planeNormal);
		planeInConvex.basis.transform(tmp);

		Vector3 vtx = convexShape?.localGetSupportingVertex(tmp, Vector3.zero()) ?? Vector3.zero();
		Vector3 vtxInPlane = Vector3.copy(vtx);
		convexInPlaneTrans.transform(vtxInPlane);

		double distance = (planeNormal.dot(vtxInPlane) - planeConstant);

		Vector3 vtxInPlaneProjected = Vector3.zero();
		tmp.scaleFrom(distance, planeNormal);
		vtxInPlaneProjected.sub2(vtxInPlane, tmp);

		Vector3 vtxInPlaneWorld = Vector3.copy(vtxInPlaneProjected);
		planeObj?.getWorldTransform(tmpTrans).transform(vtxInPlaneWorld);

		hasCollision = distance < (_manifoldPtr?.getContactBreakingThreshold() ?? 0);
		resultOut?.setPersistentManifold(_manifoldPtr);
		if (hasCollision) {
			// report a contact. internally this will be kept persistent, and contact reduction is done
			Vector3 normalOnSurfaceB = Vector3.copy(planeNormal);
			planeObj?.getWorldTransform(tmpTrans).basis.transform(normalOnSurfaceB);

			Vector3 pOnB = Vector3.copy(vtxInPlaneWorld);
			resultOut?.addContactPoint(normalOnSurfaceB, pOnB, distance);
		}
		if (_ownManifold) {
			if ((_manifoldPtr?.getNumContacts() ?? 0) != 0) {
				resultOut?.refreshContactPoints();
			}
		}
	}

	@override
	double calculateTimeOfImpact(CollisionObject? body0, CollisionObject? body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
		// not yet
		return 1;
	}

	@override
	void getAllContactManifolds(ObjectArrayList<PersistentManifold?> manifoldArray) {
		if (_manifoldPtr != null && _ownManifold) {
			manifoldArray.add(_manifoldPtr);
		}
	}
}

////////////////////////////////////////////////////////////////////////////
class CPCAFunc extends CollisionAlgorithmCreateFunc {

  @override
  CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
    ConvexPlaneCollisionAlgorithm algo = ConvexPlaneCollisionAlgorithm();
    if (!swapped) {
      algo.init(null, ci, body0, body1, false);
    }
    else {
      algo.init(null, ci, body0, body1, true);
    }
    return algo;
  }

  @override
  void releaseCollisionAlgorithm(CollisionAlgorithm algo) {}
}
