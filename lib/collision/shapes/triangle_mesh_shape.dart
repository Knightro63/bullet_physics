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

import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/internal_triangle_index_callback.dart";
import "package:bullet_physics/collision/shapes/striding_mesh_interface.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/linearmath/aabb_util2.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * Concave triangle mesh abstract class. Use {@link BvhTriangleMeshShape} as concrete
 * implementation.
 * 
 * @author jezek2
 */
abstract class TriangleMeshShape extends ConcaveShape {
	final Vector3 localAabbMin = Vector3.zero();
	final Vector3 localAabbMax = Vector3.zero();
	StridingMeshInterface? meshInterface;

	/**
	 * TriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
	 * Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
	 */
	TriangleMeshShape(this.meshInterface);
	
	Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
		Vector3 tmp = Vector3.zero();

		Vector3 supportVertex = out;

		Transform ident = Transform();
		ident.setIdentity();

		_SupportVertexCallback supportCallback = _SupportVertexCallback(vec, ident);

		Vector3 aabbMax = Vector3.zero();
		aabbMax.setValues(1e30, 1e30, 1e30);
		tmp.negateFrom(aabbMax);

		processAllTriangles(supportCallback, tmp, aabbMax);

		supportCallback.getSupportVertexLocal(supportVertex);

		return out;
	}

	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		assert (false);
		return localGetSupportingVertex(vec, out);
	}

	void recalcLocalAabb() {
		for (int i = 0; i < 3; i++) {
			Vector3 vec = Vector3.zero();
			VectorUtil.setCoord(vec, i, 1);
			Vector3 tmp = localGetSupportingVertex(vec, Vector3.zero());
			VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmp, i) + collisionMargin);
			VectorUtil.setCoord(vec, i, -1);
			localGetSupportingVertex(vec, tmp);
			VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmp, i) - collisionMargin);
		}
	}

	@override
	void getAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 tmp = Vector3.zero();

		Vector3 localHalfExtents = Vector3.zero();
		localHalfExtents.sub2(localAabbMax, localAabbMin);
		localHalfExtents.scale(0.5);

		Vector3 localCenter = Vector3.zero();
		localCenter.add2(localAabbMax, localAabbMin);
		localCenter.scale(0.5);

		Matrix3 absB = Matrix3.copy(trans.basis);
		MatrixUtil.absolute(absB);

		Vector3 center = Vector3.copy(localCenter);
		trans.transform(center);

		Vector3 extent = Vector3.zero();
		absB.getRowWith(0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		absB.getRowWith(1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		absB.getRowWith(2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		Vector3 margin = Vector3.zero();
		margin.setValues(getMargin(), getMargin(), getMargin());
		extent.add(margin);

		aabbMin.sub2(center, extent);
		aabbMax.add2(center, extent);
	}

	@override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax) {
		_FilteredCallback filterCallback = _FilteredCallback(callback, aabbMin, aabbMax);
		meshInterface?.internalProcessAllTriangles(filterCallback, aabbMin, aabbMax);
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		// moving concave objects not supported
		assert (false);
		inertia.setValues(0, 0, 0);
	}


	@override
	void setLocalScaling(Vector3 scaling) {
		meshInterface?.setScaling(scaling);
		recalcLocalAabb();
	}

	@override
	Vector3 getLocalScaling(Vector3 out) {
		return meshInterface?.getScaling(out) ?? out.clone();
	}
	
	StridingMeshInterface? getMeshInterface() {
		return meshInterface;
	}

	Vector3 getLocalAabbMin(Vector3 out) {
		out.setFrom(localAabbMin);
		return out;
	}

	Vector3 getLocalAabbMax(Vector3 out) {
		out.setFrom(localAabbMax);
		return out;
	}

	@override
	String getName() {
		return "TRIANGLEMESH";
	}
}

////////////////////////////////////////////////////////////////////////////

class _SupportVertexCallback extends TriangleCallback {
  final Vector3 _supportVertexLocal = Vector3.zero();
  final Transform worldTrans = Transform();
  double maxDot = -1e30;
  final Vector3 supportVecLocal = Vector3.zero();

  _SupportVertexCallback(Vector3 supportVecWorld,Transform trans) {
    worldTrans.copy(trans);
    MatrixUtil.transposeTransform(supportVecLocal, supportVecWorld, worldTrans.basis);
  }
  @override
  void processTriangle(List<Vector3> triangle, int partId, int triangleIndex) {
    for (int i = 0; i < 3; i++) {
      double dot = supportVecLocal.dot(triangle[i]);
      if (dot > maxDot) {
        maxDot = dot;
        _supportVertexLocal.setFrom(triangle[i]);
      }
    }
  }

  Vector3 getSupportVertexWorldSpace(Vector3 out) {
    out.setFrom(_supportVertexLocal);
    worldTrans.transform(out);
    return out;
  }

  Vector3 getSupportVertexLocal(Vector3 out) {
    out.setFrom(_supportVertexLocal);
    return out;
  }
}
class _FilteredCallback extends InternalTriangleIndexCallback {
  TriangleCallback? callback;
  final Vector3 aabbMin = Vector3.zero();
  final Vector3 aabbMax = Vector3.zero();

  _FilteredCallback(this.callback, Vector3 aabbMin, Vector3 aabbMax) {
    this.aabbMin.setFrom(aabbMin);
    this.aabbMax.setFrom(aabbMax);
  }
  @override
  void internalProcessTriangleIndex(List<Vector3> triangle, int partId, int triangleIndex) {
    if (AabbUtil2.testTriangleAgainstAabb2(triangle, aabbMin, aabbMax)) {
      // check aabb in triangle-space, before doing this
      callback?.processTriangle(triangle, partId, triangleIndex);
    }
  }
}