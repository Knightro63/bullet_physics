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

import 'package:bullet_physics/linearmath/vector_util.dart';
import 'package:bullet_physics/utils/object_array_list.dart';
import 'package:vector_math/vector_math.dart';

/**
 * GeometryUtil helper class provides a few methods to convert between plane
 * equations and vertices.
 * 
 * @author jezek2
 */
class GeometryUtil {

	static bool isPointInsidePlanes(ObjectArrayList<Vector4> planeEquations, Vector3 point, double margin) {
		int numbrushes = planeEquations.size;
		for (int i = 0; i < numbrushes; i++) {
			Vector4 n1 = planeEquations[i] ?? Vector4.zero();
			double dist = VectorUtil.dot3(n1, point) + n1.w - margin;
			if (dist > 0) {
				return false;
			}
		}
		return true;
	}
	
	static bool areVerticesBehindPlane(Vector4 planeNormal, ObjectArrayList<Vector3> vertices, double margin) {
		int numvertices = vertices.size;
		for (int i = 0; i < numvertices; i++) {
			Vector3 n1 = vertices[i] ?? Vector3.zero();
			double dist = VectorUtil.dot3(planeNormal, n1) + planeNormal.w - margin;
			if (dist > 0) {
				return false;
			}
		}
		return true;
	}
	
	static bool _notExist(Vector4 planeEquation, ObjectArrayList<Vector4> planeEquations) {
		int numbrushes = planeEquations.size;
		for (int i = 0; i < numbrushes; i++) {
			Vector4 n1 = planeEquations[i] ?? Vector4.zero();
			if (VectorUtil.dot3(planeEquation, n1) > 0.999) {
				return false;
			}
		}
		return true;
	}

	static void getPlaneEquationsFromVertices(ObjectArrayList<Vector3> vertices, ObjectArrayList<Vector4> planeEquationsOut) {
		Vector4 planeEquation = Vector4.zero();
		Vector3 edge0 = Vector3.zero(), edge1 = Vector3.zero();
		Vector3 tmp = Vector3.zero();

		int numvertices = vertices.size;
		// brute force:
		for (int i = 0; i < numvertices; i++) {
			Vector3 n1 = vertices[i] ?? Vector3.zero();

			for (int j = i + 1; j < numvertices; j++) {
				Vector3 n2 = vertices[j] ?? Vector3.zero();

				for (int k = j + 1; k < numvertices; k++) {
					Vector3 n3 = vertices[k] ?? Vector3.zero();

					edge0.sub2(n2,n1);
					edge1.sub2(n3,n1);
					double normalSign = 1;
					for (int ww = 0; ww < 2; ww++) {
						tmp.cross2(edge0, edge1);
						planeEquation.x = normalSign * tmp.x;
						planeEquation.y = normalSign * tmp.y;
						planeEquation.z = normalSign * tmp.z;

						if (VectorUtil.lengthSquared3(planeEquation) > 0.0001) {
							VectorUtil.normalize3(planeEquation);
							if(_notExist(planeEquation, planeEquationsOut)) {
								planeEquation.w = -VectorUtil.dot3(planeEquation, n1);

								// check if inside, and replace supportingVertexOut if needed
								if (areVerticesBehindPlane(planeEquation, vertices, 0.01)) {
									planeEquationsOut.add(Vector4.copy(planeEquation));
								}
							}
						}
						normalSign = -1;
					}
				}
			}
		}
	}
	
	static void getVerticesFromPlaneEquations(ObjectArrayList<Vector4> planeEquations, ObjectArrayList<Vector3> verticesOut) {
		Vector3 n2n3 = Vector3.zero();
		Vector3 n3n1 = Vector3.zero();
		Vector3 n1n2 = Vector3.zero();
		Vector3 potentialVertex = Vector3.zero();

		int numbrushes = planeEquations.size;
		// brute force:
		for (int i = 0; i < numbrushes; i++) {
			Vector4 n1 = planeEquations[i] ?? Vector4.zero();

			for (int j = i + 1; j < numbrushes; j++) {
				Vector4 n2 = planeEquations[j] ?? Vector4.zero();

				for (int k = j + 1; k < numbrushes; k++) {
					Vector4 n3 = planeEquations[k] ?? Vector4.zero();

					VectorUtil.cross3(n2n3, n2, n3);
					VectorUtil.cross3(n3n1, n3, n1);
					VectorUtil.cross3(n1n2, n1, n2);

					if ((n2n3.length2 > 0.0001) &&
							(n3n1.length2 > 0.0001) &&
							(n1n2.length2 > 0.0001)) {
						// point P out of 3 plane equations:

						// 	     d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )  
						// P =  -------------------------------------------------------------------------  
						//    N1 . ( N2 * N3 )  

						double quotient = VectorUtil.dot3(n1, n2n3);
						if (quotient.abs() > 0.000001) {
							quotient = -1 / quotient;
							n2n3.scale(n1.w);
							n3n1.scale(n2.w);
							n1n2.scale(n3.w);
							potentialVertex.setFrom(n2n3);
							potentialVertex.add(n3n1);
							potentialVertex.add(n1n2);
							potentialVertex.scale(quotient);

							// check if inside, and replace supportingVertexOut if needed
							if (isPointInsidePlanes(planeEquations, potentialVertex, 0.01)) {
								verticesOut.add(Vector3.copy(potentialVertex));
							}
						}
					}
				}
			}
		}
	}
	
}
