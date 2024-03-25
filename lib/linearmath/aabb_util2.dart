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

import 'package:bullet_physics/linearmath/matrix_util.dart';
import 'package:bullet_physics/linearmath/transform.dart';
import 'package:bullet_physics/linearmath/vector_util.dart';
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 * Utility functions for axis aligned bounding boxes (AABB).
 * 
 * @author jezek2
 */
class AabbUtil2 {

	static void aabbExpand(Vector3 aabbMin, Vector3 aabbMax, Vector3 expansionMin, Vector3 expansionMax) {
		aabbMin.add(expansionMin);
		aabbMax.add(expansionMax);
	}

	static int outcode(Vector3 p, Vector3 halfExtent) {
		return (p.x < -halfExtent.x ? 0x01 : 0x0) |
				(p.x > halfExtent.x ? 0x08 : 0x0) |
				(p.y < -halfExtent.y ? 0x02 : 0x0) |
				(p.y > halfExtent.y ? 0x10 : 0x0) |
				(p.z < -halfExtent.z ? 0x4 : 0x0) |
				(p.z > halfExtent.z ? 0x20 : 0x0);
	}
	
	static bool rayAabb(Vector3 rayFrom, Vector3 rayTo, Vector3 aabbMin, Vector3 aabbMax, double param, Vector3 normal) {
		Vector3 aabbHalfExtent = Vector3.zero();
		Vector3 aabbCenter = Vector3.zero();
		Vector3 source = Vector3.zero();
		Vector3 target = Vector3.zero();
		Vector3 r = Vector3.zero();
		Vector3 hitNormal = Vector3.zero();

		aabbHalfExtent.sub2(aabbMax,aabbMin);
		aabbHalfExtent.scale(0.5);

		aabbCenter.add2(aabbMax,aabbMin);
		aabbCenter.scale(0.5);

		source.sub2(rayFrom,aabbCenter);
		target.sub2(rayTo,aabbCenter);

		int sourceOutcode = outcode(source, aabbHalfExtent);
		int targetOutcode = outcode(target, aabbHalfExtent);
		if ((sourceOutcode & targetOutcode) == 0x0) {
			double lambdaEnter = 0;
			double lambdaExit = param;
			r.sub2(target,source);

			double normSign = 1;
			hitNormal.setValues(0, 0, 0);
			int bit = 1;

			for (int j = 0; j < 2; j++) {
				for (int i = 0; i != 3; ++i) {
					if ((sourceOutcode & bit) != 0) {
						double lambda = (-VectorUtil.getCoord(source, i) - VectorUtil.getCoord(aabbHalfExtent, i) * normSign) / VectorUtil.getCoord(r, i);
						if (lambdaEnter <= lambda) {
							lambdaEnter = lambda;
							hitNormal.setValues(0, 0, 0);
							VectorUtil.setCoord(hitNormal, i, normSign);
						}
					}
					else if ((targetOutcode & bit) != 0) {
						double lambda = (-VectorUtil.getCoord(source, i) - VectorUtil.getCoord(aabbHalfExtent, i) * normSign) / VectorUtil.getCoord(r, i);
						//btSetMin(lambda_exit, lambda);
						lambdaExit = min(lambdaExit, lambda);
					}
					bit <<= 1;
				}
				normSign = -1;
			}
			if (lambdaEnter <= lambdaExit) {
				param = lambdaEnter;
				normal.setFrom(hitNormal);
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Conservative test for overlap between two AABBs.
	 */
	static bool testAabbAgainstAabb2(Vector3 aabbMin1, Vector3 aabbMax1, Vector3 aabbMin2, Vector3 aabbMax2) {
		bool overlap = true;
		overlap = (aabbMin1.x > aabbMax2.x || aabbMax1.x < aabbMin2.x) ? false : overlap;
		overlap = (aabbMin1.z > aabbMax2.z || aabbMax1.z < aabbMin2.z) ? false : overlap;
		overlap = (aabbMin1.y > aabbMax2.y || aabbMax1.y < aabbMin2.y) ? false : overlap;
		return overlap;
	}
	
	/**
	 * Conservative test for overlap between triangle and AABB.
	 */
	static bool testTriangleAgainstAabb2(List<Vector3> vertices, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 p1 = vertices[0];
		Vector3 p2 = vertices[1];
		Vector3 p3 = vertices[2];

		if (min(min(p1.x, p2.x), p3.x) > aabbMax.x) return false;
		if (max(max(p1.x, p2.x), p3.x) < aabbMin.x) return false;

		if (min(min(p1.z, p2.z), p3.z) > aabbMax.z) return false;
		if (max(max(p1.z, p2.z), p3.z) < aabbMin.z) return false;

		if (min(min(p1.y, p2.y), p3.y) > aabbMax.y) return false;
		if (max(max(p1.y, p2.y), p3.y) < aabbMin.y) return false;
		
		return true;
	}

	static void transformAabb(Vector3 halfExtents, double margin, Transform t, Vector3 aabbMinOut, Vector3 aabbMaxOut) {
		Vector3 halfExtentsWithMargin = Vector3.zero();
		halfExtentsWithMargin.x = halfExtents.x + margin;
		halfExtentsWithMargin.y = halfExtents.y + margin;
		halfExtentsWithMargin.z = halfExtents.z + margin;

		Matrix3 absB = Matrix3.copy(t.basis);
		MatrixUtil.absolute(absB);

		Vector3 tmp = Vector3.zero();

		Vector3 center = Vector3.copy(t.origin);
		Vector3 extent = Vector3.zero();
		tmp.setFrom(absB.getRow(0));
		extent.x = tmp.dot(halfExtentsWithMargin);
		tmp.setFrom(absB.getRow(1));
		extent.y = tmp.dot(halfExtentsWithMargin);
		tmp.setFrom(absB.getRow(2));
		extent.z = tmp.dot(halfExtentsWithMargin);

		aabbMinOut.sub2(center,extent);
		aabbMaxOut.add2(center,extent);
	}

	static void transformAabbLocal(Vector3 localAabbMin, Vector3 localAabbMax, double margin, Transform trans, Vector3 aabbMinOut, Vector3 aabbMaxOut) {
		assert (localAabbMin.x <= localAabbMax.x);
		assert (localAabbMin.y <= localAabbMax.y);
		assert (localAabbMin.z <= localAabbMax.z);

		Vector3 localHalfExtents = Vector3.zero();
		localHalfExtents.sub2(localAabbMax,localAabbMin);
		localHalfExtents.scale(0.5);

		localHalfExtents.x += margin;
		localHalfExtents.y += margin;
		localHalfExtents.z += margin;

		Vector3 localCenter = Vector3.zero();
		localCenter.add(localAabbMax-localAabbMin);
		localCenter.scale(0.5);

		Matrix3 absB = Matrix3.copy(trans.basis);
		MatrixUtil.absolute(absB);

		Vector3 center = Vector3.copy(localCenter);
		trans.transform(center);

		Vector3 extent = Vector3.zero();
		Vector3 tmp = Vector3.zero();

		tmp.setFrom(absB.getRow(0));
		extent.x = tmp.dot(localHalfExtents);
		tmp.setFrom(absB.getRow(1));
		extent.y = tmp.dot(localHalfExtents);
		tmp.setFrom(absB.getRow(2));
		extent.z = tmp.dot(localHalfExtents);

		aabbMinOut.sub2(center,extent);
		aabbMaxOut.add2(center,extent);
	}
}
