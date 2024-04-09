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

import "package:bullet_physics/collision/dispatch/collision_flags.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/narrowphase/discrete_collision_detector_interface.dart";
import "package:bullet_physics/collision/narrowphase/manifold_point.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class ManifoldResult extends Result{
	PersistentManifold? _manifoldPtr;
	final Transform _rootTransA = Transform();
	final Transform _rootTransB = Transform();
	CollisionObject? _body0;
	CollisionObject? _body1;
	int _partId0 = 0;
	int _partId1 = 0;
	int _index0 = 0;
	int _index1 = 0;

	ManifoldResult([CollisionObject? body0, CollisionObject? body1]) {
		init(body0, body1);
	}

	void init(CollisionObject? body0, CollisionObject? body1) {
		_body0 = body0;
		_body1 = body1;
		body0?.getWorldTransform(_rootTransA);
		body1?.getWorldTransform(_rootTransB);
	}

	PersistentManifold? getPersistentManifold() {
		return _manifoldPtr;
	}

	void setPersistentManifold(PersistentManifold? manifoldPtr) {
		_manifoldPtr = manifoldPtr;
	}
  @override
	void setShapeIdentifiers(int partId0, int index0, int partId1, int index1) {
		_partId0 = partId0;
		_partId1 = partId1;
		_index0 = index0;
		_index1 = index1;
	}
  @override
	void addContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, double depth) {
		assert (_manifoldPtr != null);
		//order in manifold needs to match

		if (depth > _manifoldPtr!.getContactBreakingThreshold()) {
			return;
		}

		bool isSwapped = _manifoldPtr!.getBody0() != _body0;

		Vector3 pointA = Vector3.zero();
		pointA.scaleAdd(depth, normalOnBInWorld, pointInWorld);

		Vector3 localA = Vector3.zero();
		Vector3 localB = Vector3.zero();

		if (isSwapped) {
			_rootTransB.invXform(pointA, localA);
			_rootTransA.invXform(pointInWorld, localB);
		}
		else {
			_rootTransA.invXform(pointA, localA);
			_rootTransB.invXform(pointInWorld, localB);
		}

		ManifoldPoint newPt = ManifoldPoint();
		newPt.init(localA, localB, normalOnBInWorld, depth);

		newPt.positionWorldOnA.setFrom(pointA);
		newPt.positionWorldOnB.setFrom(pointInWorld);

		int insertIndex = _manifoldPtr!.getCacheEntry(newPt);

		newPt.combinedFriction = _calculateCombinedFriction(_body0, _body1);
		newPt.combinedRestitution = _calculateCombinedRestitution(_body0, _body1);

		// BP mod, store contact triangles.
		newPt.partId0 = _partId0;
		newPt.partId1 = _partId1;
		newPt.index0 = _index0;
		newPt.index1 = _index1;

		/// todo, check this for any side effects
		if (insertIndex >= 0) {
			_manifoldPtr?.replaceContactPoint(newPt, insertIndex);
		}
		else {
			insertIndex = _manifoldPtr!.addManifoldPoint(newPt);
		}
		// User can override friction and/or restitution
		if (BulletGlobals.getContactAddedCallback() != null &&
				// and if either of the two bodies requires custom material
				(((_body0?.getCollisionFlags() ?? 0) & CollisionFlags.customMaterialCallback) != 0 ||
				((_body1?.getCollisionFlags() ?? 0) & CollisionFlags.customMaterialCallback) != 0)) {
			//experimental feature info, for per-triangle material etc.
			CollisionObject? obj0 = isSwapped ? _body1 : _body0;
			CollisionObject? obj1 = isSwapped ? _body0 : _body1;
			BulletGlobals.getContactAddedCallback()?.contactAdded(
        _manifoldPtr!.getContactPoint(insertIndex), 
        obj0, 
        _partId0, 
        _index0, 
        obj1, 
        _partId1, 
        _index1
      );
		}

	}

	static double _calculateCombinedFriction(CollisionObject? body0, CollisionObject? body1) {
		double friction = (body0?.getFriction() ?? 0) * (body1?.getFriction() ?? 0);

		double maxFriction = 10;
		if (friction < -maxFriction) {
			friction = -maxFriction;
		}
		if (friction > maxFriction) {
			friction = maxFriction;
		}
		return friction;
	}

	static double _calculateCombinedRestitution(CollisionObject? body0, CollisionObject? body1) {
		return (body0?.getRestitution() ?? 0) * (body1?.getRestitution() ?? 0);
	}

	void refreshContactPoints() {
		assert (_manifoldPtr != null);
		if (_manifoldPtr!.getNumContacts() == 0) {
			return;
		}

		bool isSwapped = _manifoldPtr!.getBody0() != _body0;

		if (isSwapped) {
			_manifoldPtr!.refreshContactPoints(_rootTransB, _rootTransA);
		}
		else {
			_manifoldPtr!.refreshContactPoints(_rootTransA, _rootTransB);
		}
	}
}
