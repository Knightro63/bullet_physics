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
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class GeometryOperations {

	static const double planeDirEpsilon = 0.0000001;
	static const double paraleleNormals = 0.000001;
	
	static double clamp(double number, double minval, double maxval) {
		return (number < minval? minval : (number > maxval? maxval : number));
	}

	/**
	 * Calc a plane from a triangle edge an a normal.
	 */
	static void edgePlane(Vector3 e1, Vector3 e2, Vector3 normal, Vector4 plane) {
		Vector3 planenormal = Vector3.zero();
		planenormal.sub2(e2, e1);
		planenormal.cross2(planenormal, normal);
		planenormal.normalize();

		plane.setFromVector3(planenormal);
		plane.w = e2.dot(planenormal);
	}
	
	/**
	 * Finds the closest point(cp) to (v) on a segment (e1,e2).
	 */
	static void closestPointOnSegment(Vector3 cp, Vector3 v, Vector3 e1, Vector3 e2) {
		Vector3 n = Vector3.zero();
		n.sub2(e2, e1);
		cp.sub2(v, e1);
		double _scalar = cp.dot(n) / n.dot(n);
		if (_scalar < 0.0) {
			cp = e1;
		}
		else if (_scalar > 1.0) {
			cp = e2;
		}
		else {
			cp.scaleAdd(_scalar, n, e1);
		}
	}
	
	/**
	 * Line plane collision.
	 * 
	 * @return -0 if the ray never intersects, -1 if the ray collides in front, -2 if the ray collides in back
	 */
	static int linePlaneCollision(Vector4 plane, Vector3 vDir, Vector3 vPoint, Vector3 pout, List<double> tparam, double tmin, double tmax) {
		double _dotdir = VectorUtil.dot3(vDir, plane);

		if (_dotdir.abs() < planeDirEpsilon) {
			tparam[0] = tmax;
			return 0;
		}

		double _dis = ClipPolygon.distancePointPlane(plane, vPoint);
		int returnvalue = _dis < 0.0 ? 2 : 1;
		tparam[0] = -_dis / _dotdir;

		if (tparam[0] < tmin) {
			returnvalue = 0;
			tparam[0] = tmin;
		}
		else if (tparam[0] > tmax) {
			returnvalue = 0;
			tparam[0] = tmax;
		}
		pout.scaleAdd(tparam[0], vDir, vPoint);
		return returnvalue;
	}
	
	/**
	 * Find closest points on segments.
	 */
	static void segmentCollision(Vector3 vA1, Vector3 vA2, Vector3 vB1, Vector3 vB2, Vector3 vPointA, Vector3 vPointB) {
		Vector3 AD = Vector3.zero();
		AD.sub2(vA2, vA1);

		Vector3 BD = Vector3.zero();
		BD.sub2(vB2, vB1);

		Vector3 N = Vector3.zero();
		N.cross2(AD, BD);
		List<double> tp = [N.length2];

		Vector4 _M = Vector4.zero();//plane

		if (tp[0] < BulletGlobals.simdEpsilon)//ARE PARALELE
		{
			// project B over A
			bool invert_b_order = false;
			_M.x = vB1.dot(AD);
			_M.y = vB2.dot(AD);

			if (_M.x > _M.y) {
				invert_b_order = true;
				//BT_SWAP_NUMBERS(_M[0],_M[1]);
				_M.x = _M.x + _M.y;
				_M.y = _M.x - _M.y;
				_M.x = _M.x - _M.y;
			}
			_M.z = vA1.dot(AD);
			_M.w = vA2.dot(AD);
			// mid points
			N.x = (_M.x + _M.y) * 0.5;
			N.y = (_M.z + _M.w) * 0.5;

			if (N.x < N.y) {
				if (_M.y < _M.z) {
					vPointB = invert_b_order ? vB1 : vB2;
					vPointA = vA1;
				}
				else if (_M.y < _M.w) {
					vPointB = invert_b_order ? vB1 : vB2;
					closestPointOnSegment(vPointA, vPointB, vA1, vA2);
				}
				else {
					vPointA = vA2;
					closestPointOnSegment(vPointB, vPointA, vB1, vB2);
				}
			}
			else {
				if (_M.w < _M.x) {
					vPointB = invert_b_order ? vB2 : vB1;
					vPointA = vA2;
				}
				else if (_M.w < _M.y) {
					vPointA = vA2;
					closestPointOnSegment(vPointB, vPointA, vB1, vB2);
				}
				else {
					vPointB = invert_b_order ? vB1 : vB2;
					closestPointOnSegment(vPointA, vPointB, vA1, vA2);
				}
			}
			return;
		}

		N.cross2(N, BD);
		_M.setValues(N.x, N.y, N.z, vB1.dot(N));

		// get point A as the plane collision point
		linePlaneCollision(_M, AD, vA1, vPointA, tp, 0, 1);

		/*Closest point on segment*/
		vPointB.sub2(vPointA, vB1);
		tp[0] = vPointB.dot(BD);
		tp[0] /= BD.dot(BD);
		tp[0] = clamp(tp[0], 0.0, 1.0);

		vPointB.scaleAdd(tp[0], BD, vB1);
	}
	
}
