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
import "package:bullet_physics/collision/shapes/bvh_triangle_mesh_shape.dart";
import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class ScaledBvhTriangleMeshShape extends ConcaveShape {
 
	final Vector3 localScaling = Vector3.zero();
	late BvhTriangleMeshShape bvhTriMeshShape;

	ScaledBvhTriangleMeshShape(BvhTriangleMeshShape childShape, Vector3 localScaling) {
		this.localScaling.setFrom(localScaling);
		bvhTriMeshShape = childShape;
	}

	BvhTriangleMeshShape getChildShape() {
		return bvhTriMeshShape;
	}

	@override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax) {
		_ScaledTriangleCallback scaledCallback = _ScaledTriangleCallback(callback, localScaling);

		Vector3 invLocalScaling = Vector3.zero();
		invLocalScaling.setValues(1/ localScaling.x, 1 / localScaling.y, 1 / localScaling.z);

		Vector3 scaledAabbMin = Vector3.zero();
		Vector3 scaledAabbMax = Vector3.zero();

		// support negative scaling
		scaledAabbMin.x = localScaling.x >= 0 ? aabbMin.x * invLocalScaling.x : aabbMax.x * invLocalScaling.x;
		scaledAabbMin.y = localScaling.y >= 0 ? aabbMin.y * invLocalScaling.y : aabbMax.y * invLocalScaling.y;
		scaledAabbMin.z = localScaling.z >= 0 ? aabbMin.z * invLocalScaling.z : aabbMax.z * invLocalScaling.z;

		scaledAabbMax.x = localScaling.x <= 0 ? aabbMin.x * invLocalScaling.x : aabbMax.x * invLocalScaling.x;
		scaledAabbMax.y = localScaling.y <= 0 ? aabbMin.y * invLocalScaling.y : aabbMax.y * invLocalScaling.y;
		scaledAabbMax.z = localScaling.z <= 0 ? aabbMin.z * invLocalScaling.z : aabbMax.z * invLocalScaling.z;

		bvhTriMeshShape.processAllTriangles(scaledCallback, scaledAabbMin, scaledAabbMax);
	}

	@override
	void getAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 localAabbMin = bvhTriMeshShape.getLocalAabbMin(Vector3.zero());
		Vector3 localAabbMax = bvhTriMeshShape.getLocalAabbMax(Vector3.zero());

		Vector3 tmpLocalAabbMin = Vector3.zero();
		Vector3 tmpLocalAabbMax = Vector3.zero();
		VectorUtil.mul(tmpLocalAabbMin, localAabbMin, localScaling);
		VectorUtil.mul(tmpLocalAabbMax, localAabbMax, localScaling);

		localAabbMin.x = (localScaling.x >= 0) ? tmpLocalAabbMin.x : tmpLocalAabbMax.x;
		localAabbMin.y = (localScaling.y >= 0) ? tmpLocalAabbMin.y : tmpLocalAabbMax.y;
		localAabbMin.z = (localScaling.z >= 0) ? tmpLocalAabbMin.z : tmpLocalAabbMax.z;
		localAabbMax.x = (localScaling.x <= 0) ? tmpLocalAabbMin.x : tmpLocalAabbMax.x;
		localAabbMax.y = (localScaling.y <= 0) ? tmpLocalAabbMin.y : tmpLocalAabbMax.y;
		localAabbMax.z = (localScaling.z <= 0) ? tmpLocalAabbMin.z : tmpLocalAabbMax.z;

		Vector3 localHalfExtents = Vector3.zero();
		localHalfExtents.sub2(localAabbMax, localAabbMin);
		localHalfExtents.scale(0.5);

		double margin = bvhTriMeshShape.getMargin();
		localHalfExtents.x += margin;
		localHalfExtents.y += margin;
		localHalfExtents.z += margin;

		Vector3 localCenter = Vector3.zero();
		localCenter.add2(localAabbMax, localAabbMin);
		localCenter.scale(0.5);

		Matrix3 absB = Matrix3.copy(trans.basis);
		MatrixUtil.absolute(absB);

		Vector3 center = Vector3.copy(localCenter);
		trans.transform(center);

		Vector3 extent = Vector3.zero();
		Vector3 tmp = Vector3.zero();
		absB.getRowWith(0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		absB.getRowWith(1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		absB.getRowWith(2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		aabbMin.sub2(center, extent);
		aabbMax.add2(center, extent);
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.scaledTriangleMeshShapeProxytype;
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
	}

	@override
	String getName() {
		return "SCALEDBVHTRIANGLEMESH";
	}
}

////////////////////////////////////////////////////////////////////////////

class _ScaledTriangleCallback extends TriangleCallback {
  late TriangleCallback? _originalCallback;
  late Vector3 _localScaling;
  final List<Vector3> _newTriangle = [Vector3.zero(),Vector3.zero(),Vector3.zero()];//Vector3[3];

  _ScaledTriangleCallback(TriangleCallback? originalCallback, Vector3 localScaling) {
    _originalCallback = originalCallback;
    _localScaling = localScaling;
  }
  @override
  void processTriangle(List<Vector3> triangle, int partId, int triangleIndex) {
    VectorUtil.mul(_newTriangle[0], triangle[0], _localScaling);
    VectorUtil.mul(_newTriangle[1], triangle[1], _localScaling);
    VectorUtil.mul(_newTriangle[2], triangle[2], _localScaling);
    _originalCallback?.processTriangle(_newTriangle, partId, triangleIndex);
  }
}
