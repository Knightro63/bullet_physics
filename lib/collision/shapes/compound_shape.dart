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

import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/compound_shape_child.dart";
import "package:bullet_physics/collision/shapes/optimized_bvh.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

class CompoundShape extends CollisionShape {

	final ObjectArrayList<CompoundShapeChild> _children = ObjectArrayList();
	final Vector3 _localAabbMin = Vector3(1e30, 1e30, 1e30);
	final Vector3 _localAabbMax = Vector3(-1e30, -1e30, -1e30);

	OptimizedBvh? _aabbTree;

	double _collisionMargin = 0;
	final Vector3 localScaling = Vector3(1, 1, 1);

	void addChildShape(Transform localTransform, CollisionShape shape) {
		CompoundShapeChild child = CompoundShapeChild();
		child.transform.copy(localTransform);
		child.childShape = shape;
		child.childShapeType = shape.getShapeType();
		child.childMargin = shape.getMargin();

		_children.add(child);

		// extend the local aabbMin/aabbMax
		Vector3 localAabbMin = Vector3.zero(), localAabbMax = Vector3.zero();
		shape.getAabb(localTransform, _localAabbMin, _localAabbMax);
		VectorUtil.setMin(_localAabbMin, localAabbMin);
		VectorUtil.setMax(_localAabbMax, localAabbMax);
	}

	/**
	 * Remove all children shapes that contain the specified shape.
	 */
	void removeChildShape(CollisionShape shape) {
		bool done_removing;

		// Find the children containing the shape specified, and remove those children.
		do {
			done_removing = true;

			for (int i = 0; i < _children.size; i++) {
				if (_children.getQuick(i)?.childShape == shape) {
					_children.removeAt(i);
					done_removing = false;  // Do another iteration pass after removing from the vector
					break;
				}
			}
		}
		while (!done_removing);

		recalculateLocalAabb();
	}
	
	int getNumChildShapes() {
		return _children.size;
	}

	CollisionShape? getChildShape(int index) {
		return _children.getQuick(index)?.childShape;
	}

	Transform getChildTransform(int index, Transform out) {
		out.copy(_children.getQuick(index)!.transform);
		return out;
	}

	ObjectArrayList<CompoundShapeChild> getChildList() {
		return _children;
	}

	/**
	 * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
	 */
	@override
	void getAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 localHalfExtents = Vector3.zero();
		localHalfExtents.sub2(_localAabbMax, _localAabbMin);
		localHalfExtents.scale(0.5);
		localHalfExtents.x += getMargin();
		localHalfExtents.y += getMargin();
		localHalfExtents.z += getMargin();

		Vector3 localCenter = Vector3.zero();
		localCenter.add2(_localAabbMax, _localAabbMin);
		localCenter.scale(0.5);

		Matrix3 abs_b = Matrix3.copy(trans.basis);
		MatrixUtil.absolute(abs_b);

		Vector3 center = Vector3.copy(localCenter);
		trans.transform(center);

		Vector3 tmp = Vector3.zero();

		Vector3 extent = Vector3.zero();
		abs_b.getRowWith(0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		abs_b.getRowWith(1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		abs_b.getRowWith(2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		aabbMin.sub2(center, extent);
		aabbMax.add2(center, extent);
	}

	/**
	 * Re-calculate the local Aabb. Is called at the end of removeChildShapes.
	 * Use this yourself if you modify the children or their transforms.
	 */
	void recalculateLocalAabb() {
		// Recalculate the local aabb
		// Brute force, it iterates over all the shapes left.
		_localAabbMin.setValues(1e30, 1e30, 1e30);
		_localAabbMax.setValues(-1e30, -1e30, -1e30);

		Vector3 tmpLocalAabbMin = Vector3.zero();
		Vector3 tmpLocalAabbMax = Vector3.zero();

		// extend the local aabbMin/aabbMax
		for (int j = 0; j < _children.size; j++) {
			_children.getQuick(j)?.childShape?.getAabb(_children.getQuick(j)!.transform, tmpLocalAabbMin, tmpLocalAabbMax);
			
			for (int i = 0; i < 3; i++) {
				if (VectorUtil.getCoord(_localAabbMin, i) > VectorUtil.getCoord(tmpLocalAabbMin, i)) {
					VectorUtil.setCoord(_localAabbMin, i, VectorUtil.getCoord(tmpLocalAabbMin, i));
				}
				if (VectorUtil.getCoord(_localAabbMax, i) < VectorUtil.getCoord(tmpLocalAabbMax, i)) {
					VectorUtil.setCoord(_localAabbMax, i, VectorUtil.getCoord(tmpLocalAabbMax, i));
				}
			}
		}
	}
	
	@override
	void setLocalScaling(Vector3 scaling) {
		localScaling.setFrom(scaling);
	}

	@override
	Vector3 getLocalScaling(Vector3 out) {
		out.setFrom(localScaling);
		return out;
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		// approximation: take the inertia from the aabb for now
		Transform ident = Transform();
		ident.setIdentity();
		Vector3 aabbMin = Vector3.zero(), aabbMax = Vector3.zero();
		getAabb(ident, aabbMin, aabbMax);

		Vector3 halfExtents = Vector3.zero();
		halfExtents.sub2(aabbMax, aabbMin);
		halfExtents.scale(0.5);

		double lx = 2 * halfExtents.x;
		double ly = 2 * halfExtents.y;
		double lz = 2 * halfExtents.z;

		inertia.x = (mass / 12) * (ly * ly + lz * lz);
		inertia.y = (mass / 12) * (lx * lx + lz * lz);
		inertia.z = (mass / 12) * (lx * lx + ly * ly);
	}
	
	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.compoundShapeProxyType;
	}

	@override
	void setMargin(double margin) {
		_collisionMargin = margin;
	}

	@override
	double getMargin() {
		return _collisionMargin;
	}

	@override
	String getName() {
		return "Compound";
	}

	// this is optional, but should make collision queries faster, by culling non-overlapping nodes
	// void	createAabbTreeFromChildren();
	
	OptimizedBvh? getAabbTree() {
		return _aabbTree;
	}

	/**
	 * Computes the exact moment of inertia and the transform from the coordinate
	 * system defined by the principal axes of the moment of inertia and the center
	 * of mass to the current coordinate system. "masses" points to an array
	 * of masses of the children. The resulting transform "principal" has to be
	 * applied inversely to all children transforms in order for the local coordinate
	 * system of the compound shape to be centered at the center of mass and to coincide
	 * with the principal axes. This also necessitates a correction of the world transform
	 * of the collision object by the principal transform.
	 */
	void calculatePrincipalAxisTransform(List<double> masses, Transform principal, Vector3 inertia) {
		int n = _children.size;

		double totalMass = 0;
		Vector3 center = Vector3.zero();
		center.setValues(0, 0, 0);
		for (int k = 0; k < n; k++) {
			center.scaleAdd(masses[k], _children.getQuick(k)?.transform.origin ?? Vector3.zero(), center);
			totalMass += masses[k];
		}
		center.scale(1 / totalMass);
		principal.origin.setFrom(center);

		Matrix3 tensor = Matrix3.zero();
		tensor.setZero();

		for (int k = 0; k < n; k++) {
			Vector3 i = Vector3.zero();
			_children.getQuick(k)?.childShape?.calculateLocalInertia(masses[k], i);

			Transform? t = _children.getQuick(k)?.transform;
			Vector3 o = Vector3.zero();
			o.sub2(t?.origin, center);

			// compute inertia tensor in coordinate system of compound shape
			Matrix3 j = Matrix3.zero();
			j.transposeFrom(t!.basis);

			j.storage[0] *= i.x;
			j.storage[1] *= i.x;
			j.storage[2] *= i.x;
			j.storage[3] *= i.y;
			j.storage[4] *= i.y;
			j.storage[5] *= i.y;
			j.storage[6] *= i.z;
			j.storage[7] *= i.z;
			j.storage[8] *= i.z;

			j.mul2(t.basis, j);

			// add inertia tensor
			tensor.add(j);

			// compute inertia tensor of pointmass at o
			double o2 = o.length2;
			j.setRowByValues(0, o2, 0, 0);
			j.setRowByValues(1, 0, o2, 0);
			j.setRowByValues(2, 0, 0, o2);
			j.storage[0] += o.x * -o.x;
			j.storage[1] += o.y * -o.x;
			j.storage[2] += o.z * -o.x;
			j.storage[3] += o.x * -o.y;
			j.storage[4] += o.y * -o.y;
			j.storage[5] += o.z * -o.y;
			j.storage[6] += o.x * -o.z;
			j.storage[7] += o.y * -o.z;
			j.storage[8] += o.z * -o.z;

			// add inertia tensor of pointmass
			tensor.storage[0] += masses[k] * j.storage[0];
			tensor.storage[1] += masses[k] * j.storage[1];
			tensor.storage[2] += masses[k] * j.storage[2];
			tensor.storage[3] += masses[k] * j.storage[3];
			tensor.storage[4] += masses[k] * j.storage[4];
			tensor.storage[5] += masses[k] * j.storage[5];
			tensor.storage[6] += masses[k] * j.storage[6];
			tensor.storage[7] += masses[k] * j.storage[7];
			tensor.storage[8] += masses[k] * j.storage[8];
		}

		MatrixUtil.diagonalize(tensor, principal.basis, 0.00001, 20);

		inertia.setValues(tensor.storage[0], tensor.storage[4], tensor.storage[8]);
	}

}
