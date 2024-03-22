/*
 * Dart port of Bullet (c) 2024 @Knightro63
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

import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

class ClipPolygon {
	
	static double distancePointPlane(Vector4 plane, Vector3 point) {
		return VectorUtil.dot3(point, plane) - plane.w;
	}

	/**
	 * Vector blending. Takes two vectors a, b, blends them together.
	 */
	static void vecBlend(Vector3 vr, Vector3 va, Vector3 vb, double blendFactor) {
		vr.scaleFrom(1 - blendFactor, va);
		vr.scaleAdd(blendFactor, vb, vr);
	}

	/**
	 * This function calcs the distance from a 3D plane.
	 */
	static void planeClipPolygonCollect(Vector3 point0, Vector3 point1, double dist0, double dist1, ObjectArrayList<Vector3> clipped, int clippedCount) {
		bool prevclassif = (dist0 > BulletGlobals.simdEpsilon);
		bool classif = (dist1 > BulletGlobals.simdEpsilon);
		if (classif != prevclassif) {
			double blendfactor = -dist0 / (dist1 - dist0);
			vecBlend(clipped.getQuick(clippedCount) ?? Vector3.zero(), point0, point1, blendfactor);
			clippedCount++;
		}
		if (!classif) {
			clipped.getQuick(clippedCount)?.setFrom(point1);
			clippedCount++;
		}
	}

	/**
	 * Clips a polygon by a plane.
	 * 
	 * @return The count of the clipped counts
	 */
	static int planeClipPolygon(Vector4 plane, ObjectArrayList<Vector3> polygonPoints, int polygonPointCount, ObjectArrayList<Vector3> clipped) {
		//List<int> intArrays = List.filled(length, fill);//ArrayPool.get(int.class);

		int clippedCount = 0;//intArrays.getFixed(1);

		// clip first point
		double firstdist = distancePointPlane(plane, polygonPoints.getQuick(0) ?? Vector3.zero());
		if (!(firstdist > BulletGlobals.simdEpsilon)) {
			clipped.getQuick(clippedCount)?.setFrom(polygonPoints.getQuick(0) ?? Vector3.zero());
			clippedCount++;
		}

		double olddist = firstdist;
		for (int i=1; i<polygonPointCount; i++) {
			double dist = distancePointPlane(plane, polygonPoints.getQuick(i) ?? Vector3.zero());
			planeClipPolygonCollect(
        polygonPoints.getQuick(i - 1) ?? Vector3.zero(), 
        polygonPoints.getQuick(i) ?? Vector3.zero(),
        olddist,
        dist,
        clipped,
        clippedCount
      );
			olddist = dist;
		}

		// RETURN TO FIRST point

		planeClipPolygonCollect(
      polygonPoints.getQuick(polygonPointCount - 1) ?? Vector3.zero(), 
      polygonPoints.getQuick(0) ?? Vector3.zero(),
      olddist,
      firstdist,
      clipped,
      clippedCount
    );

		int ret = clippedCount;
		//intArrays.release(clippedCount);
		return ret;
	}

	/**
	 * Clips a polygon by a plane.
	 * 
	 * @param clipped must be an array of 16 points.
	 * @return the count of the clipped counts
	 */
	static int planeClipTriangle(Vector4 plane, Vector3 point0, Vector3 point1, Vector3 point2, ObjectArrayList<Vector3> clipped) {
		//ArrayPool<List<int>> intArrays = ArrayPool.get(int.class);

		int clippedCount = 0;//intArrays.getFixed(1);

		// clip first point0
		double firstdist = distancePointPlane(plane, point0);
		if (!(firstdist > BulletGlobals.simdEpsilon)) {
			clipped.getQuick(clippedCount)?.setFrom(point0);
			clippedCount++;
		}

		// point 1
		double olddist = firstdist;
		double dist = distancePointPlane(plane, point1);

		planeClipPolygonCollect(
				point0, point1,
				olddist,
				dist,
				clipped,
				clippedCount);

		olddist = dist;


		// point 2
		dist = distancePointPlane(plane, point2);

		planeClipPolygonCollect(
				point1, point2,
				olddist,
				dist,
				clipped,
				clippedCount);
		olddist = dist;



		// RETURN TO FIRST point0
		planeClipPolygonCollect(
				point2, point0,
				olddist,
				firstdist,
				clipped,
				clippedCount);

		int ret = clippedCount;
		//intArrays.release(clippedCount);
		return ret;
	}
	
}
