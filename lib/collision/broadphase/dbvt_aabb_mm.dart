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

// Dbvt implementation by Nathanael Presson

import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class DbvtAabbMm {

	final Vector3 _mi = Vector3.zero();
	final Vector3 _mx = Vector3.zero();

	DbvtAabbMm([DbvtAabbMm? o]) {
    if(o != null){
		  set(o);
    }
	}
	
	void set(DbvtAabbMm o) {
		_mi.setFrom(o._mi);
		_mx.setFrom(o._mx);
	}
	
	static void swap(DbvtAabbMm p1, DbvtAabbMm? p2) {
    if(p2 == null) return;
		Vector3 tmp = Vector3.zero();
		
		tmp.setFrom(p1._mi);
		p1._mi.setFrom(p2._mi);
		p2._mi.setFrom(tmp);

		tmp.setFrom(p1._mx);
		p1._mx.setFrom(p2._mx);
		p2._mx.setFrom(tmp);
	}

	Vector3 center(Vector3 out) {
		out.add2(_mi, _mx);
		out.scale(0.5);
		return out;
	}
	
	Vector3 lengths(Vector3 out) {
		out.sub2(_mx,_mi);
		return out;
	}
	
	Vector3 extents(Vector3 out) {
		out.sub2(_mx,_mi);
		out.scale(0.5);
		return out;
	}
	
	Vector3 mins() {
		return _mi;
	}

	Vector3 maxs() {
		return _mx;
	}
	
	static DbvtAabbMm fromCE(Vector3 c, Vector3 e, DbvtAabbMm out) {
		DbvtAabbMm box = out;
		box._mi.sub2(c,e);
		box._mx.add2(c,e);
		return box;
	}

	static DbvtAabbMm fromCR(Vector3 c, double r, DbvtAabbMm out) {
		Vector3 tmp = Vector3.zero();
		tmp.setValues(r, r, r);
		return fromCE(c, tmp, out);
	}

	static DbvtAabbMm fromMM(Vector3 mi, Vector3 mx, DbvtAabbMm out) {
		DbvtAabbMm box = out;
		box._mi.setFrom(mi);
		box._mx.setFrom(mx);
		return box;
	}
	
	//static  DbvtAabbMm	FromPoints( btVector3* pts,int n);
	//static  DbvtAabbMm	FromPoints( btVector3** ppts,int n);
	
	void expand(Vector3 e) {
		_mi.sub(e);
		_mx.add(e);
	}

	void signedExpand(Vector3 e) {
		if (e.x > 0) {
			_mx.x += e.x;
		}
		else {
			_mi.x += e.x;
		}
		
		if (e.y > 0) {
			_mx.y += e.y;
		}
		else {
			_mi.y += e.y;
		}
		
		if (e.z > 0) {
			_mx.z += e.z;
		}
		else {
			_mi.z += e.z;
		}
	}

	bool contain(DbvtAabbMm? a) {
    if(a == null) return false;
		return ((_mi.x <= a._mi.x) &&
		        (_mi.y <= a._mi.y) &&
		        (_mi.z <= a._mi.z) &&
		        (_mx.x >= a._mx.x) &&
		        (_mx.y >= a._mx.y) &&
		        (_mx.z >= a._mx.z));
	}

	int classify(Vector3 n, double o, int s) {
		Vector3 pi = Vector3.zero();
		Vector3 px = Vector3.zero();

		switch (s) {
			case (0):
				px.setValues(_mi.x, _mi.y, _mi.z);
				pi.setValues(_mx.x, _mx.y, _mx.z);
				break;
			case (1):
				px.setValues(_mx.x, _mi.y, _mi.z);
				pi.setValues(_mi.x, _mx.y, _mx.z);
				break;
			case (2):
				px.setValues(_mi.x, _mx.y, _mi.z);
				pi.setValues(_mx.x, _mi.y, _mx.z);
				break;
			case (3):
				px.setValues(_mx.x, _mx.y, _mi.z);
				pi.setValues(_mi.x, _mi.y, _mx.z);
				break;
			case (4):
				px.setValues(_mi.x, _mi.y, _mx.z);
				pi.setValues(_mx.x, _mx.y, _mi.z);
				break;
			case (5):
				px.setValues(_mx.x, _mi.y, _mx.z);
				pi.setValues(_mi.x, _mx.y, _mi.z);
				break;
			case (6):
				px.setValues(_mi.x, _mx.y, _mx.z);
				pi.setValues(_mx.x, _mi.y, _mi.z);
				break;
			case (7):
				px.setValues(_mx.x, _mx.y, _mx.z);
				pi.setValues(_mi.x, _mi.y, _mi.z);
				break;
		}
		
		if ((n.dot(px) + o) < 0) {
			return -1;
		}
		if ((n.dot(pi) + o) >= 0) {
			return 1;
		}
		return 0;
	}

	double projectMinimum(Vector3 v, int signs) {
		List<Vector3> b = [_mx,_mi];
		Vector3 p = Vector3.zero();
		p.setValues(b[(signs >> 0) & 1].x,
		      b[(signs >> 1) & 1].y,
		      b[(signs >> 2) & 1].z);
		return p.dot(v);
	}
	 
	static bool intersect(DbvtAabbMm a, DbvtAabbMm b) {
		return ((a._mi.x <= b._mx.x) &&
		        (a._mx.x >= b._mi.x) &&
		        (a._mi.y <= b._mx.y) &&
		        (a._mx.y >= b._mi.y) &&
		        (a._mi.z <= b._mx.z) &&
		        (a._mx.z >= b._mi.z));
	}

	static bool intersectWithTransform(DbvtAabbMm? a, DbvtAabbMm? b, Transform xform) {
    if(a == null || b == null) return false;
		Vector3 d0 = Vector3.zero();
		Vector3 d1 = Vector3.zero();
		Vector3 tmp = Vector3.zero();

		// JAVA NOTE: check
		b.center(d0);
		xform.transform(d0);
		d0.sub(a.center(tmp));

		MatrixUtil.transposeTransform(d1, d0, xform.basis);

		List<double> s0 = [0,0];
		List<double> s1 = [0,0];
		s1[0] = xform.origin.dot(d0);
		s1[1] = s1[0];

		a._addSpan(d0, s0, 0, s0, 1);
		b._addSpan(d1, s1, 0, s1, 1);
		if (s0[0] > (s1[1])) {
			return false;
		}
		if (s0[1] < (s1[0])) {
			return false;
		}
		return true;
	}

	static bool intersectWithVector(DbvtAabbMm a, Vector3 b) {
		return ((b.x >= a._mi.x) &&
		        (b.y >= a._mi.y) &&
		        (b.z >= a._mi.z) &&
		        (b.x <= a._mx.x) &&
		        (b.y <= a._mx.y) &&
		        (b.z <= a._mx.z));
	}

	static bool intersectWithSigns(DbvtAabbMm? a, Vector3 org, Vector3 invdir, List<int> signs) {
    if(a == null) return false;
		List<Vector3> bounds = [a._mi, a._mx];
		double txmin = (bounds[signs[0]].x - org.x) * invdir.x;
		double txmax = (bounds[1 - signs[0]].x - org.x) * invdir.x;
		double tymin = (bounds[signs[1]].y - org.y) * invdir.y;
		double tymax = (bounds[1 - signs[1]].y - org.y) * invdir.y;
		if ((txmin > tymax) || (tymin > txmax)) {
			return false;
		}
		
		if (tymin > txmin) {
			txmin = tymin;
		}
		if (tymax < txmax) {
			txmax = tymax;
		}
		double tzmin = (bounds[signs[2]].z - org.z) * invdir.z;
		double tzmax = (bounds[1 - signs[2]].z - org.z) * invdir.z;
		if ((txmin > tzmax) || (tzmin > txmax)) {
			return false;
		}
		
		if (tzmin > txmin) {
			txmin = tzmin;
		}
		if (tzmax < txmax) {
			txmax = tzmax;
		}
		return (txmax > 0);
	}

	static double proximity(DbvtAabbMm? a, DbvtAabbMm? b) {
    if(a == null || b == null) return 0;
		Vector3 d = Vector3.zero();
		Vector3 tmp = Vector3.zero();

		d.add2(a._mi, a._mx);
		tmp.add2(b._mi, b._mx);
		d.sub(tmp);
		return d.x.abs() + d.y.abs() + d.z.abs();
	}

	static void merge(DbvtAabbMm a, DbvtAabbMm b, DbvtAabbMm r) {
		for (int i=0; i<3; i++) {
			if (VectorUtil.getCoord(a._mi, i) < VectorUtil.getCoord(b._mi, i)) {
				VectorUtil.setCoord(r._mi, i, VectorUtil.getCoord(a._mi, i));
			}
			else {
				VectorUtil.setCoord(r._mi, i, VectorUtil.getCoord(b._mi, i));
			}
			
			if (VectorUtil.getCoord(a._mx, i) > VectorUtil.getCoord(b._mx, i)) {
				VectorUtil.setCoord(r._mx, i, VectorUtil.getCoord(a._mx, i));
			}
			else {
				VectorUtil.setCoord(r._mx, i, VectorUtil.getCoord(b._mx, i));
			}
		}
	}

	static bool notEqual(DbvtAabbMm a, DbvtAabbMm b) {
		return ((a._mi.x != b._mi.x) ||
		        (a._mi.y != b._mi.y) ||
		        (a._mi.z != b._mi.z) ||
		        (a._mx.x != b._mx.x) ||
		        (a._mx.y != b._mx.y) ||
		        (a._mx.z != b._mx.z));
	}
	
	void _addSpan(Vector3 d, List<double> smi, int smiIdx, List<double> smx, int smxIdx) {
		for (int i=0; i<3; i++) {
			if (VectorUtil.getCoord(d, i) < 0) {
				smi[smiIdx] += VectorUtil.getCoord(_mx, i) * VectorUtil.getCoord(d, i);
				smx[smxIdx] += VectorUtil.getCoord(_mi, i) * VectorUtil.getCoord(d, i);
			}
			else {
				smi[smiIdx] += VectorUtil.getCoord(_mi, i) * VectorUtil.getCoord(d, i);
				smx[smxIdx] += VectorUtil.getCoord(_mx, i) * VectorUtil.getCoord(d, i);
			}
		}
	}
}
