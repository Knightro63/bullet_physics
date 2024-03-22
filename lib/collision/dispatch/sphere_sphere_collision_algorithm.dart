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

import "package:bullet_physics/collision/broadphase/collision_algorithm.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm_construction_info.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/collision_algorithm_create_func.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/shapes/sphere_shape.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

/**
 * Provides collision detection between two spheres.
 * 
 * @author jezek2
 */
class SphereSphereCollisionAlgorithm extends CollisionAlgorithm {
  static SSCAFunc CreateFunction() => SSCAFunc();
	bool _ownManifold = false;
	PersistentManifold? _manifoldPtr;
	
	SphereSphereCollisionAlgorithm([
    PersistentManifold? mf, 
    CollisionAlgorithmConstructionInfo? ci, 
    CollisionObject? col0, 
    CollisionObject? col1
  ]){
		init(mf, ci, col0, col1);
	}

  void init(    
    PersistentManifold? mf, 
    CollisionAlgorithmConstructionInfo? ci, 
    CollisionObject? col0, 
    CollisionObject? col1
  ){
    initCA(ci);
		_manifoldPtr = mf;

		if (_manifoldPtr == null) {
			_manifoldPtr = dispatcher?.getNewManifold(col0, col1);
			_ownManifold = true;
		}
  }
	
	@override
	void destroy() {
		if (_ownManifold) {
			if (_manifoldPtr != null) {
				dispatcher?.releaseManifold(_manifoldPtr!);
			}
			_manifoldPtr = null;
		}
	}
	
	@override
	void processCollision(CollisionObject? body0, CollisionObject? body1, DispatcherInfo? dispatchInfo, ManifoldResult? resultOut) {
		if (_manifoldPtr == null) {
			return;
		}
		
		Transform tmpTrans1 = Transform();
		Transform tmpTrans2 = Transform();

		resultOut?.setPersistentManifold(_manifoldPtr!);

		SphereShape? sphere0 = body0?.getCollisionShape() as SphereShape?;
		SphereShape? sphere1 = body1?.getCollisionShape() as SphereShape?;

		Vector3 diff = Vector3.zero();
		diff.sub2(body0?.getWorldTransform(tmpTrans1).origin, body1?.getWorldTransform(tmpTrans2).origin);

		double len = diff.length;
		double radius0 = sphere0?.getRadius() ?? 0;
		double radius1 = sphere1?.getRadius() ?? 0;

		//#ifdef CLEAR_MANIFOLD
		//manifoldPtr.clearManifold(); // don't do this, it disables warmstarting
		//#endif

		// if distance positive, don't generate a contact
		if (len > (radius0 + radius1)) {
			//#ifndef CLEAR_MANIFOLD
			resultOut?.refreshContactPoints();
			//#endif //CLEAR_MANIFOLD
			return;
		}
		// distance (negative means penetration)
		double dist = len - (radius0 + radius1);

		Vector3 normalOnSurfaceB = Vector3.zero();
		normalOnSurfaceB.setValues(1, 0, 0);
		if (len > BulletGlobals.fltEpsilon) {
			normalOnSurfaceB.scaleFrom(1 / len, diff);
		}

		Vector3 tmp = Vector3.zero();

		// point on A (worldspace)
		Vector3 pos0 = Vector3.zero();
		tmp.scaleFrom(radius0, normalOnSurfaceB);
		pos0.sub2(body0?.getWorldTransform(tmpTrans1).origin, tmp);

		// point on B (worldspace)
		Vector3 pos1 = Vector3.zero();
		tmp.scaleFrom(radius1, normalOnSurfaceB);
		pos1.add2(body1?.getWorldTransform(tmpTrans2).origin, tmp);

		// report a contact. internally this will be kept persistent, and contact reduction is done
		resultOut?.addContactPoint(normalOnSurfaceB, pos1, dist);

		//#ifndef CLEAR_MANIFOLD
		resultOut?.refreshContactPoints();
		//#endif //CLEAR_MANIFOLD
	}

	@override
	double calculateTimeOfImpact(CollisionObject? body0, CollisionObject? body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
		return 1;
	}

	@override
	void getAllContactManifolds(ObjectArrayList<PersistentManifold> manifoldArray) {
		if (_manifoldPtr != null && _ownManifold) {
			manifoldArray.add(_manifoldPtr!);
		}
	}
}

////////////////////////////////////////////////////////////////////////////

class SSCAFunc extends CollisionAlgorithmCreateFunc {
  @override
  CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
    SphereSphereCollisionAlgorithm algo = SphereSphereCollisionAlgorithm();
    algo.init(null, ci, body0, body1);
    return algo;
  }

  @override
  void releaseCollisionAlgorithm(CollisionAlgorithm algo) {

  }
}