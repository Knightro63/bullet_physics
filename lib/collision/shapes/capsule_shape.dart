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

import "package:bullet_physics/collision/shapes/convex_internal_shape.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

class CapsuleShape extends ConvexInternalShape {
	int upAxis = 1;
	
	CapsuleShape([double? radius, double? height]) {
    if(radius != null && height != null){
		  implicitShapeDimensions.setValues(radius, 0.5 * height, radius);
    }
	}

	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec0, Vector3 out) {
		Vector3 supVec = out;
		supVec.setValues(0, 0, 0);

		double maxDot = -1e30;

		Vector3 vec = Vector3.copy(vec0);
		double lenSqr = vec.length2;
		if (lenSqr < 0.0001) {
			vec.setValues(1, 0, 0);
		}
		else {
			double rlen = 1 / sqrt(lenSqr);
			vec.scale(rlen);
		}

		Vector3 vtx = Vector3.zero();
		double newDot;

		double radius = getRadius();

		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();
		Vector3 pos = Vector3.zero();

		{
			pos.setValues(0, 0, 0);
			VectorUtil.setCoord(pos, getUpAxis(), getHalfHeight());
			
			VectorUtil.mul(tmp1, vec, localScaling);
			tmp1.scale(radius);
			tmp2.scaleFrom(getMargin(), vec);
			vtx.add2(pos, tmp1);
			vtx.sub(tmp2);
			newDot = vec.dot(vtx);
			if (newDot > maxDot) {
				maxDot = newDot;
				supVec.setFrom(vtx);
			}
		}
		{
			pos.setValues(0, 0, 0);
			VectorUtil.setCoord(pos, getUpAxis(), -getHalfHeight());
			
			VectorUtil.mul(tmp1, vec, localScaling);
			tmp1.scale(radius);
			tmp2.scaleFrom(getMargin(), vec);
			vtx.add2(pos, tmp1);
			vtx.sub(tmp2);
			newDot = vec.dot(vtx);
			if (newDot > maxDot) {
				maxDot = newDot;
				supVec.setFrom(vtx);
			}
		}

		return out;
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		// TODO: implement
		throw("Not supported yet.");
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		// as an approximation, take the inertia of the box that bounds the spheres

		Transform ident = Transform();
		ident.setIdentity();

		double radius = getRadius();

		Vector3 halfExtents = Vector3.zero();
		halfExtents.setValues(radius, radius, radius);
		VectorUtil.setCoord(halfExtents, getUpAxis(), radius + getHalfHeight());

		double margin = BulletGlobals.convexDistanceMargin;

		double lx = 2 * (halfExtents.x + margin);
		double ly = 2 * (halfExtents.y + margin);
		double lz = 2 * (halfExtents.z + margin);
		double x2 = lx * lx;
		double y2 = ly * ly;
		double z2 = lz * lz;
		double scaledmass = mass * 0.08333333;

		inertia.x = scaledmass * (y2 + z2);
		inertia.y = scaledmass * (x2 + z2);
		inertia.z = scaledmass * (x2 + y2);
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.capsuleShapeProxytype;
	}
	
	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 tmp = Vector3.zero();

		Vector3 halfExtents = Vector3.zero();
		halfExtents.setValues(getRadius(), getRadius(), getRadius());
		VectorUtil.setCoord(halfExtents, upAxis, getRadius() + getHalfHeight());

		halfExtents.x += getMargin();
		halfExtents.y += getMargin();
		halfExtents.z += getMargin();

		Matrix3 abs_b = Matrix3.zero();
		abs_b.setFrom(t.basis);
		MatrixUtil.absolute(abs_b);

		Vector3 center = t.origin;
		Vector3 extent = Vector3.zero();

		abs_b.getRowWith(0, tmp);
		extent.x = tmp.dot(halfExtents);
		abs_b.getRowWith(1, tmp);
		extent.y = tmp.dot(halfExtents);
		abs_b.getRowWith(2, tmp);
		extent.z = tmp.dot(halfExtents);

		aabbMin.sub2(center, extent);
		aabbMax.add2(center, extent);
	}

	@override
	String getName() {
		return "CapsuleShape";
	}
	
	int getUpAxis() {
		return upAxis;
	}
	
	double getRadius() {
		int radiusAxis = (upAxis + 2) % 3;
		return VectorUtil.getCoord(implicitShapeDimensions, radiusAxis);
	}

	double getHalfHeight() {
		return VectorUtil.getCoord(implicitShapeDimensions, upAxis);
	}

}
