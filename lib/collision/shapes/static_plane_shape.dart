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
import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * StaticPlaneShape simulates an infinite non-moving (static) collision plane.
 * 
 * @author jezek2
 */
class StaticPlaneShape extends ConcaveShape {

	final Vector3 localAabbMin = Vector3.zero();
	final Vector3 localAabbMax = Vector3.zero();
	
	final Vector3 planeNormal = Vector3.zero();
	double planeConstant;
	final Vector3 localScaling = Vector3(0, 0, 0);

	StaticPlaneShape(Vector3 planeNormal, this.planeConstant) {
		this.planeNormal.normalizeFrom(planeNormal);
	}

	Vector3 getPlaneNormal(Vector3 out) {
		out.setFrom(planeNormal);
		return out;
	}

	double getPlaneConstant() {
		return planeConstant;
	}
	
	@override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 tmp = Vector3.zero();
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		Vector3 halfExtents = Vector3.zero();
		halfExtents.sub2(aabbMax, aabbMin);
		halfExtents.scale(0.5);

		double radius = halfExtents.length;
		Vector3 center = Vector3.zero();
		center.add2(aabbMax, aabbMin);
		center.scale(0.5);

		// this is where the triangles are generated, given AABB and plane equation (normal/constant)

		Vector3 tangentDir0 = Vector3.zero(), tangentDir1 = Vector3.zero();

		// tangentDir0/tangentDir1 can be precalculated
		TransformUtil.planeSpace1(planeNormal, tangentDir0, tangentDir1);

		//Vector3 supVertex0 = Vector3.zero(), supVertex1 = Vector3.zero();

		Vector3 projectedCenter = Vector3.zero();
		tmp.scaleFrom(planeNormal.dot(center) - planeConstant, planeNormal);
		projectedCenter.sub2(center, tmp);

		List<Vector3> triangle = [ Vector3.zero(), Vector3.zero(), Vector3.zero() ];

		tmp1.scaleFrom(radius, tangentDir0);
		tmp2.scaleFrom(radius, tangentDir1);
		VectorUtil.add(triangle[0], projectedCenter, tmp1, tmp2);

		tmp1.scaleFrom(radius, tangentDir0);
		tmp2.scaleFrom(radius, tangentDir1);
		tmp.sub2(tmp1, tmp2);
		VectorUtil.add(triangle[1], projectedCenter, tmp);

		tmp1.scaleFrom(radius, tangentDir0);
		tmp2.scaleFrom(radius, tangentDir1);
		tmp.sub2(tmp1, tmp2);
		triangle[2].sub2(projectedCenter, tmp);

		callback?.processTriangle(triangle, 0, 0);

		tmp1.scaleFrom(radius, tangentDir0);
		tmp2.scaleFrom(radius, tangentDir1);
		tmp.sub2(tmp1, tmp2);
		triangle[0].sub2(projectedCenter, tmp);

		tmp1.scaleFrom(radius, tangentDir0);
		tmp2.scaleFrom(radius, tangentDir1);
		tmp.add2(tmp1, tmp2);
		triangle[1].sub2(projectedCenter, tmp);

		tmp1.scaleFrom(radius, tangentDir0);
		tmp2.scaleFrom(radius, tangentDir1);
		VectorUtil.add(triangle[2], projectedCenter, tmp1, tmp2);

		callback?.processTriangle(triangle, 0, 1);
	}

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		aabbMin.setValues(-1e30, -1e30, -1e30);
		aabbMax.setValues(1e30, 1e30, 1e30);
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.staticPlaneProxytype;
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
		//moving concave objects not supported
		inertia.setValues(0, 0, 0);
	}

	@override
	String getName() {
		return "STATICPLANE";
	}

}
