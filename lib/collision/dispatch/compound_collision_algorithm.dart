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

import "package:bullet_physics/collision/dispatch/collision_algorithm_create_func.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm_construction_info.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/compound_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/utils/object_array_list.dart";

/**
 * CompoundCollisionAlgorithm supports collision between {@link CompoundShape}s and
 * other collision shapes.
 * 
 * @author jezek2
 */
class CompoundCollisionAlgorithm extends CollisionAlgorithm {
  static CCA2Func CreateFunction() => CCA2Func();
  static SwappedCCA2Func SwappedCreateFunction() => SwappedCCA2Func();

	final ObjectArrayList<CollisionAlgorithm?> _childCollisionAlgorithms = ObjectArrayList();
	bool _isSwapped = false;
	
	void init(CollisionAlgorithmConstructionInfo ci, [CollisionObject? body0, CollisionObject? body1, bool isSwapped = false]) {
		initCA(ci);
		
		_isSwapped = isSwapped;

		CollisionObject? colObj = _isSwapped ? body1 : body0;
		CollisionObject? otherObj = _isSwapped ? body0 : body1;
		assert (colObj?.getCollisionShape()?.isCompound() ?? false);

		CompoundShape? compoundShape = colObj?.getCollisionShape() as CompoundShape?;
		int numChildren = compoundShape?.getNumChildShapes() ?? 0;
		int i = 0;

		//childCollisionAlgorithms.resize(numChildren);
		for (i = 0; i < numChildren; i++) {
			CollisionShape? tmpShape = colObj?.getCollisionShape();
			CollisionShape? childShape = compoundShape?.getChildShape(i);
			colObj?.internalSetTemporaryCollisionShape(childShape);
			_childCollisionAlgorithms.add(ci.dispatcher1?.findAlgorithm(colObj, otherObj));
			colObj?.internalSetTemporaryCollisionShape(tmpShape);
		}
	}

	@override
	void destroy() {
		int numChildren = _childCollisionAlgorithms.size;
		for (int i=0; i<numChildren; i++) {
			//childCollisionAlgorithms.get(i).destroy();
			dispatcher?.freeCollisionAlgorithm(_childCollisionAlgorithms.getQuick(i));
		}
		_childCollisionAlgorithms.clear();
	}
	
	@override
	void processCollision(CollisionObject? body0, CollisionObject? body1, DispatcherInfo? dispatchInfo, ManifoldResult? resultOut) {
		CollisionObject? colObj = _isSwapped ? body1 : body0;
		CollisionObject? otherObj = _isSwapped ? body0 : body1;

		assert (colObj?.getCollisionShape()?.isCompound() ?? false);
		CompoundShape? compoundShape = colObj?.getCollisionShape() as CompoundShape?;

		// We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
		// If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree traversals
		// given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
		// determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
		// then use each overlapping node AABB against Tree0
		// and vise versa.

		//Transform tmpTrans = Transform();
		Transform orgTrans = Transform();
		Transform childTrans = Transform();
		Transform orgInterpolationTrans = Transform();
		Transform newChildWorldTrans = Transform();

		int numChildren = _childCollisionAlgorithms.size;
		int i;
		for (i = 0; i < numChildren; i++) {
			// temporarily exchange parent btCollisionShape with childShape, and recurse
			CollisionShape? childShape = compoundShape?.getChildShape(i);

			// backup
			colObj?.getWorldTransform(orgTrans);
			colObj?.getInterpolationWorldTransform(orgInterpolationTrans);

			compoundShape?.getChildTransform(i, childTrans);
			newChildWorldTrans.mul(orgTrans, childTrans);
			colObj?.setWorldTransform(newChildWorldTrans);
			colObj?.setInterpolationWorldTransform(newChildWorldTrans);
			
			// the contactpoint is still projected back using the original inverted worldtrans
			CollisionShape? tmpShape = colObj?.getCollisionShape();
			colObj?.internalSetTemporaryCollisionShape(childShape);
			_childCollisionAlgorithms.getQuick(i)?.processCollision(colObj, otherObj, dispatchInfo, resultOut);
			// revert back
			colObj?.internalSetTemporaryCollisionShape(tmpShape);
			colObj?.setWorldTransform(orgTrans);
			colObj?.setInterpolationWorldTransform(orgInterpolationTrans);
		}
	}

	@override
	double calculateTimeOfImpact(CollisionObject? body0, CollisionObject? body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
		CollisionObject? colObj = _isSwapped ? body1 : body0;
		CollisionObject? otherObj = _isSwapped ? body0 : body1;

		assert (colObj?.getCollisionShape()?.isCompound() ?? false);

		CompoundShape? compoundShape = colObj?.getCollisionShape() as CompoundShape?;

		// We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
		// If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree traversals
		// given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
		// determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
		// then use each overlapping node AABB against Tree0
		// and vise versa.

		Transform tmpTrans = Transform();
		Transform orgTrans = Transform();
		Transform childTrans = Transform();
		double hitFraction = 1;

		int numChildren = _childCollisionAlgorithms.size;
		int i;
		for (i = 0; i < numChildren; i++) {
			// temporarily exchange parent btCollisionShape with childShape, and recurse
			CollisionShape? childShape = compoundShape?.getChildShape(i);

			// backup
			colObj?.getWorldTransform(orgTrans);

			compoundShape?.getChildTransform(i, childTrans);
			//btTransform	newChildWorldTrans = orgTrans*childTrans ;
			tmpTrans.copy(orgTrans);
			tmpTrans.mul(childTrans);
			colObj?.setWorldTransform(tmpTrans);

			CollisionShape? tmpShape = colObj?.getCollisionShape();
			colObj?.internalSetTemporaryCollisionShape(childShape);
			double frac = _childCollisionAlgorithms.getQuick(i)?.calculateTimeOfImpact(colObj, otherObj, dispatchInfo, resultOut) ?? 0;
			if (frac < hitFraction) {
				hitFraction = frac;
			}
			// revert back
			colObj?.internalSetTemporaryCollisionShape(tmpShape);
			colObj?.setWorldTransform(orgTrans);
		}
		return hitFraction;
	}

	@override
	void getAllContactManifolds(ObjectArrayList<PersistentManifold> manifoldArray) {
		for (int i=0; i < _childCollisionAlgorithms.size; i++) {
			_childCollisionAlgorithms.getQuick(i)?.getAllContactManifolds(manifoldArray);
		}
	}
}


	
////////////////////////////////////////////////////////////////////////////

class CCA2Func extends CollisionAlgorithmCreateFunc {

  @override
  CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
    CompoundCollisionAlgorithm algo = CompoundCollisionAlgorithm();
    algo.init(ci, body0, body1, false);
    return algo;
  }

  @override
  void releaseCollisionAlgorithm(CollisionAlgorithm algo) {

  }
}

class SwappedCCA2Func extends CollisionAlgorithmCreateFunc {
  @override
  CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
    CompoundCollisionAlgorithm algo = CompoundCollisionAlgorithm();
    algo.init(ci, body0, body1, true);
    return algo;
  }

  @override
  void releaseCollisionAlgorithm(CollisionAlgorithm algo) {

  }
}