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

import "package:bullet_physics/collision/shapes/box_shape.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

class CylinderShape extends BoxShape {
	int upAxis = 1;

	CylinderShape(super.halfExtents, [bool unused = false]) {
		recalcLocalAabb();
	}

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		polyhedralConvexShapeGetAabb(t, aabbMin, aabbMax);
	}

	Vector3 cylinderLocalSupportX(Vector3 halfExtents, Vector3 v, Vector3 out) {
		return _cylinderLocalSupport(halfExtents, v, 0, 1, 0, 2, out);
	}

	Vector3 cylinderLocalSupportY(Vector3 halfExtents, Vector3 v, Vector3 out) {
		return _cylinderLocalSupport(halfExtents, v, 1, 0, 1, 2, out);
	}

	Vector3 cylinderLocalSupportZ(Vector3 halfExtents, Vector3 v, Vector3 out) {
		return _cylinderLocalSupport(halfExtents, v, 2, 0, 2, 1, out);
	}
	
	Vector3 _cylinderLocalSupport(Vector3 halfExtents, Vector3 v, int cylinderUpAxis, int XX, int YY, int ZZ, Vector3 out) {
		//mapping depends on how cylinder local orientation is
		// extents of the cylinder is: X,Y is for radius, and Z for height

		double radius = VectorUtil.getCoord(halfExtents, XX);
		double halfHeight = VectorUtil.getCoord(halfExtents, cylinderUpAxis);

		double d;

		double s = sqrt(VectorUtil.getCoord(v, XX) * VectorUtil.getCoord(v, XX) + VectorUtil.getCoord(v, ZZ) * VectorUtil.getCoord(v, ZZ));
		if (s != 0) {
			d = radius / s;
			VectorUtil.setCoord(out, XX, VectorUtil.getCoord(v, XX) * d);
			VectorUtil.setCoord(out, YY, VectorUtil.getCoord(v, YY) < 0 ? -halfHeight : halfHeight);
			VectorUtil.setCoord(out, ZZ, VectorUtil.getCoord(v, ZZ) * d);
			return out;
		}
		else {
			VectorUtil.setCoord(out, XX, radius);
			VectorUtil.setCoord(out, YY, VectorUtil.getCoord(v, YY) < 0 ? -halfHeight : halfHeight);
			VectorUtil.setCoord(out, ZZ, 0);
			return out;
		}
	}

	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		return cylinderLocalSupportY(getHalfExtentsWithoutMargin(Vector3.zero()), vec, out);
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		for (int i = 0; i < numVectors; i++) {
			cylinderLocalSupportY(getHalfExtentsWithoutMargin(Vector3.zero()), vectors[i], supportVerticesOut[i]);
		}
	}

	@override
	Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
		Vector3 supVertex = out;
		localGetSupportingVertexWithoutMargin(vec, supVertex);

		if (getMargin() != 0) {
			Vector3 vecnorm = Vector3.copy(vec);
			if (vecnorm.length2 < (BulletGlobals.simdEpsilon * BulletGlobals.simdEpsilon)) {
				vecnorm.setValues(-1, -1, -1);
			}
			vecnorm.normalize();
			supVertex.scaleAdd(getMargin(), vecnorm, supVertex);
		}
		return out;
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.cylinderShapeProxytype;
	}

	int getUpAxis() {
		return upAxis;
	}
	
	double getRadius() {
		return getHalfExtentsWithMargin(Vector3.zero()).x;
	}

	@override
	String getName() {
		return "CylinderY";
	}
	
}
