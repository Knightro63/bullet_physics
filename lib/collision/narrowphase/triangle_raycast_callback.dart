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

import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
abstract class TriangleRaycastCallback extends TriangleCallback {
	
	//final BulletStack stack = BulletStack.get();

	final Vector3 from = Vector3.zero();
	final Vector3 to = Vector3.zero();

	double hitFraction = 1;

	TriangleRaycastCallback(Vector3 from, Vector3 to) {
		this.from.setFrom(from);
		this.to.setFrom(to);
	}
	
	void processTriangle(List<Vector3> triangle, int partId, int triangleIndex) {
		Vector3 vert0 = triangle[0];
		Vector3 vert1 = triangle[1];
		Vector3 vert2 = triangle[2];

		Vector3 v10 = Vector3.zero();
		v10.sub2(vert1,vert0);

		Vector3 v20 = Vector3.zero();
		v20.sub2(vert2,vert0);

		Vector3 triangleNormal = Vector3.zero();
		triangleNormal.cross(v10-v20);

		double dist = vert0.dot(triangleNormal);
		double distA = triangleNormal.dot(from);
		distA -= dist;
		double distB = triangleNormal.dot(to);
		distB -= dist;

		if (distA* distB >= 0) {
			return; // same sign
		}

		double proj_length = distA - distB;
		double distance = (distA) / (proj_length);
		// Now we have the intersection point on the plane, we'll see if it's inside the triangle
		// Add an epsilon as a tolerance for the raycast,
		// in case the ray hits exacly on the edge of the triangle.
		// It must be scaled for the triangle size.

		if (distance < hitFraction) {
			double edgeTolerance = triangleNormal.length2;
			edgeTolerance *= -0.0001;
			Vector3 point = Vector3.zero();
			VectorUtil.setInterpolate3(point, from, to, distance);
			{
				Vector3 v0p = Vector3.zero();
				v0p.sub2(vert0,point);
				Vector3 v1p = Vector3.zero();
				v1p.sub2(vert1,point);
				Vector3 cp0 = Vector3.zero();
				cp0.cross2(v0p, v1p);

				if (cp0.dot(triangleNormal) >= edgeTolerance) {
					Vector3 v2p = Vector3.zero();
					v2p.sub2(vert2,point);
					Vector3 cp1 = Vector3.zero();
					cp1.cross2(v1p, v2p);
					if (cp1.dot(triangleNormal) >= edgeTolerance) {
						Vector3 cp2 = Vector3.zero();
						cp2.cross2(v2p, v0p);

						if (cp2.dot(triangleNormal) >= edgeTolerance) {

							if (distA > 0) {
								hitFraction = reportHit(triangleNormal, distance, partId, triangleIndex);
							}
							else {
								Vector3 tmp = Vector3.zero();
								tmp.negateFrom(triangleNormal);
								hitFraction = reportHit(tmp, distance, partId, triangleIndex);
							}
						}
					}
				}
			}
		}
	}

	double reportHit(Vector3 hitNormalLocal, double hitFraction, int partId, int triangleIndex );
}
