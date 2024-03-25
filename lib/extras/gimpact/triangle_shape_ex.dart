/*
 * Dart port of Bullet (c) 2024 @Knightro
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
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



import "package:bullet_physics/collision/shapes/triangle_shape.dart";
import "package:bullet_physics/extras/gimpact/clip_polygon.dart";
import "package:bullet_physics/linearmath/aabb.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class TriangleShapeEx extends TriangleShape {
	TriangleShapeEx([super.p0, super.p1, super.p2]);

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 tv0 = Vector3.copy(vertices1[0]);
		t.transform(tv0);
		Vector3 tv1 = Vector3.copy(vertices1[1]);
		t.transform(tv1);
		Vector3 tv2 = Vector3.copy(vertices1[2]);
		t.transform(tv2);

		AABB trianglebox = AABB();
		trianglebox.init(tv0,tv1,tv2,collisionMargin);
		
		aabbMin.setFrom(trianglebox.min);
		aabbMax.setFrom(trianglebox.max);
	}

	void applyTransform(Transform t) {
		t.transform(vertices1[0]);
		t.transform(vertices1[1]);
		t.transform(vertices1[2]);
	}

	void buildTriPlane(Vector4 plane) {
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		Vector3 normal = Vector3.zero();
		tmp1.sub2(vertices1[1], vertices1[0]);
		tmp2.sub2(vertices1[2], vertices1[0]);
		normal.cross2(tmp1, tmp2);
		normal.normalize();

		plane.setValues(normal.x, normal.y, normal.z, vertices1[0].dot(normal));
	}

	bool overlap_test_conservative(TriangleShapeEx other) {
		double total_margin = getMargin() + other.getMargin();

		Vector4 plane0 = Vector4.zero();
		buildTriPlane(plane0);
		Vector4 plane1 = Vector4.zero();
		other.buildTriPlane(plane1);

		// classify points on other triangle
		double dis0 = ClipPolygon.distancePointPlane(plane0, other.vertices1[0]) - total_margin;

		double dis1 = ClipPolygon.distancePointPlane(plane0, other.vertices1[1]) - total_margin;

		double dis2 = ClipPolygon.distancePointPlane(plane0, other.vertices1[2]) - total_margin;

		if (dis0 > 0.0 && dis1 > 0.0 && dis2 > 0.0) {
			return false; // classify points on this triangle
		}
		dis0 = ClipPolygon.distancePointPlane(plane1, vertices1[0]) - total_margin;

		dis1 = ClipPolygon.distancePointPlane(plane1, vertices1[1]) - total_margin;

		dis2 = ClipPolygon.distancePointPlane(plane1, vertices1[2]) - total_margin;

		if (dis0 > 0.0 && dis1 > 0.0 && dis2 > 0.0) {
			return false;
		}
		return true;
	}
	
}
