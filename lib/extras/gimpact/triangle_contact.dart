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

import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/extras/gimpact/clip_polygon.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class TriangleContact {
	
	final List<List<int>> _intArrays = [];
	
	static const int maxTriClipping = 16;

    double penetrationDepth = 0;
    int pointCount = 0;
    final Vector4 separatingNormal = Vector4.zero();
    List<Vector3> points = [];

	TriangleContact(){
    for(int i = 0; i < maxTriClipping;i++){
      points.add(Vector3.zero());
    }
  }

	TriangleContact.copy(TriangleContact other) {
		copyFrom(other);
	}

	void set(TriangleContact other) {
		copyFrom(other);
	}
	
	void copyFrom(TriangleContact other) {
		penetrationDepth = other.penetrationDepth;
		separatingNormal.setFrom(other.separatingNormal);
		pointCount = other.pointCount;
		int i = pointCount;
		while ((i--) != 0) {
			points[i].setFrom(other.points[i]);
		}
	}
	
	/**
	 * Classify points that are closer.
	 */
	void mergePoints(Vector4 plane, double margin, ObjectArrayList<Vector3> points, int pointCount) {
		this.pointCount = 0;
		penetrationDepth = -1000.0;

		List<int> pointIndices = List.filled(margin.toInt(), 0);//intArrays.getFixed(margin);

		for (int k = 0; k < pointCount; k++) {
			double dist = -ClipPolygon.distancePointPlane(plane, points.getQuick(k) ?? Vector3.zero()) + margin;

			if (dist >= 0.0) {
				if (dist > penetrationDepth) {
					penetrationDepth = dist;
					pointIndices[0] = k;
					this.pointCount = 1;
				}
				else if ((dist + BulletGlobals.simdEpsilon) >= penetrationDepth) {
					pointIndices[this.pointCount] = k;
					this.pointCount++;
				}
			}
		}

		for (int k = 0; k < this.pointCount; k++) {
      if(points.getQuick(pointIndices[k]) != null){
			  this.points[k].setFrom(points.getQuick(pointIndices[k])!);
      }
		}
		
		_intArrays.remove(pointIndices);//intArrays.release(margin);
	}
}
