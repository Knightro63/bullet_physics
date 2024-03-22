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

import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/shapes/convex_internal_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * SphereShape implements an implicit sphere, centered around a local origin with radius.
 * 
 * @author jezek2
 */
class SphereShape extends ConvexInternalShape {
	
	SphereShape(double radius) {
		implicitShapeDimensions.x = radius;
		collisionMargin = radius;
	}

	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		out.setValues(0, 0, 0);
		return out;
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		for (int i = 0; i < numVectors; i++) {
			supportVerticesOut[i].setValues(0, 0, 0);
		}
	}

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 center = t.origin;
		Vector3 extent = Vector3.zero();
		extent.setValues(getMargin(), getMargin(), getMargin());
		aabbMin.sub2(center, extent);
		aabbMax.add2(center, extent);
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.sphereShapeProxytype;
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		double elem = 0.4 * mass * getMargin() * getMargin();
		inertia.setValues(elem, elem, elem);
	}

	@override
	String getName() {
		return "SPHERE";
	}
	
	double getRadius() {
		return implicitShapeDimensions.x * localScaling.x;
	}

	@override
	double getMargin() {
		// to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		// this means, non-uniform scaling is not supported anymore
		return getRadius();
	}
	
}
