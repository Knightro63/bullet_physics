/*
 * Dart port of Bullet (c) 2024 @Knightro63
 *
 * Stan Melax Convex Hull Computation
 * Copyright (c) 2024 Stan Melax http://www.melax.com/
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

// includes modifications/improvements by John Ratcliff, see BringOutYourDead below.
// import "package:bullet_physics/core/bullet_globals.dart";
// import com.bulletphysics.collision.shapes.ShapeHull;
// import "package:bullet_physics/linearmath/misc_util.dart";
// import "package:bullet_physics/linearmath/vector_util.dart";

import 'dart:math';
import 'package:bullet_physics/core/bullet_globals.dart';
import 'package:bullet_physics/linearmath/convexhull/hull_desc.dart';
import 'package:bullet_physics/linearmath/convexhull/hull_flags.dart';
import 'package:bullet_physics/linearmath/convexhull/hull_result.dart';
import 'package:bullet_physics/linearmath/convexhull/p_hull_result.dart';
import 'package:bullet_physics/linearmath/convexhull/int4.dart';
import 'package:bullet_physics/linearmath/convexhull/int3.dart';
import 'package:bullet_physics/linearmath/convexhull/tri.dart';
import 'package:bullet_physics/linearmath/misc_util.dart';
import 'package:bullet_physics/linearmath/vector_util.dart';
import 'package:bullet_physics/utils/int_array_list.dart';
import 'package:bullet_physics/utils/object_array_list.dart';
import 'package:vector_math/vector_math.dart';
/**
 * HullLibrary class can create a convex hull from a collection of vertices, using
 * the ComputeHull method. The {@link ShapeHull} class uses this HullLibrary to create
 * a approximate convex mesh given a general (non-polyhedral) convex shape.
 *
 * @author jezek2
 */
class HullLibrary {

	final IntArrayList vertexIndexMapping = IntArrayList();

	final ObjectArrayList<Tri?> _tris = ObjectArrayList();
	
	/**
	 * Converts point cloud to polygonal representation.
	 * 
	 * @param desc   describes the input request
	 * @param result contains the result
	 * @return whether conversion was successful
	 */
	bool createConvexHull(HullDesc desc, HullResult result) {
		bool ret = false;

		PHullResult hr = PHullResult();

		int vcount = desc.vcount;
		if (vcount < 8) vcount = 8;
		
		ObjectArrayList<Vector3> vertexSource = ObjectArrayList();
		MiscUtil.resizeObjectArray(vertexSource, vcount, Vector3.zero());

		Vector3 scale = Vector3.zero();

		List<int> ovcount = [];

		bool ok = _cleanupVertices(desc.vcount, desc.vertices, desc.vertexStride, ovcount, vertexSource, desc.normalEpsilon, scale); // normalize point cloud, remove duplicates!

		if (ok) {
			//		if ( 1 ) // scale vertices back to their original size.
			{
				for (int i=0; i<ovcount[0]; i++) {
					Vector3 v = vertexSource[i] ?? Vector3.zero();
					VectorUtil.mul(v, v, scale);
				}
			}

			ok = _computeHull(ovcount[0], vertexSource, hr, desc.maxVertices);

			if (ok) {
				// re-index triangle mesh so it refers to only used vertices, rebuild a vertex table.
				ObjectArrayList<Vector3> vertexScratch = ObjectArrayList();
				MiscUtil.resizeObjectArray(vertexScratch, hr.vcount, Vector3.zero());

				_bringOutYourDead(hr.vertices, hr.vcount, vertexScratch, ovcount, hr.indices, hr.indexCount);

				ret = true;

				if (desc.hasHullFlag(HullFlags.triangles)) { // if he wants the results as triangle!
					result.polygons = false;
					result.numOutputVertices = ovcount[0];
					MiscUtil.resizeObjectArray(result.outputVertices, ovcount[0], Vector3.zero());
					result.numFaces = hr.faceCount;
					result.numIndices = hr.indexCount;

					MiscUtil.resizeArray(result.indices, hr.indexCount, 0);

					for (int i=0; i<ovcount[0]; i++) {
            if(vertexScratch[i] != null){
						  result.outputVertices[i]?.setFrom(vertexScratch[i]!);
            }
					}

					if (desc.hasHullFlag(HullFlags.reverseOrder)) {
						IntArrayList sourcePtr = hr.indices;
						int sourceIdx = 0;

						IntArrayList destPtr = result.indices;
						int destIdx = 0;

						for (int i=0; i<hr.faceCount; i++) {
							destPtr.set(destIdx + 0, sourcePtr.get(sourceIdx + 2));
							destPtr.set(destIdx + 1, sourcePtr.get(sourceIdx + 1));
							destPtr.set(destIdx + 2, sourcePtr.get(sourceIdx + 0));
							destIdx += 3;
							sourceIdx += 3;
						}
					}
					else {
						for (int i=0; i<hr.indexCount; i++) {
							result.indices.set(i,hr.indices[i]);
						}
					}
				}
				else {
					result.polygons = true;
					result.numOutputVertices = ovcount[0];
					MiscUtil.resizeObjectArray(result.outputVertices, ovcount[0], Vector3.zero());
					result.numFaces = hr.faceCount;
					result.numIndices = hr.indexCount + hr.faceCount;
					MiscUtil.resizeArray(result.indices, result.numIndices, 0);
					for (int i=0; i<ovcount[0]; i++) {
            if(vertexScratch[i] != null){
						  result.outputVertices[i]?.setFrom(vertexScratch[i]!);
            }
					}

					//				if ( 1 )
					{
						IntArrayList sourcePtr = hr.indices;
						int sourceIdx = 0;

						IntArrayList destPtr = result.indices;
						int destIdx = 0;

						for (int i=0; i<hr.faceCount; i++) {
							destPtr.set(destIdx + 0, 3);
							if (desc.hasHullFlag(HullFlags.reverseOrder)) {
								destPtr.set(destIdx + 1, sourcePtr.get(sourceIdx + 2));
								destPtr.set(destIdx + 2, sourcePtr.get(sourceIdx + 1));
								destPtr.set(destIdx + 3, sourcePtr.get(sourceIdx + 0));
							}
							else {
								destPtr.set(destIdx + 1, sourcePtr.get(sourceIdx + 0));
								destPtr.set(destIdx + 2, sourcePtr.get(sourceIdx + 1));
								destPtr.set(destIdx + 3, sourcePtr.get(sourceIdx + 2));
							}

							destIdx += 4;
							sourceIdx += 3;
						}
					}
				}
				_releaseHull(hr);
			}
		}

		return ret;
	}
	
	/**
	 * Release memory allocated for this result, we are done with it.
	 */
	bool releaseResult(HullResult result) {
		if (result.outputVertices.isNotEmpty) {
			result.numOutputVertices = 0;
			result.outputVertices.clear();
		}
		if (result.indices.isNotEmpty) {
			result.numIndices = 0;
			result.indices.clear();
		}
		return true;
	}

	bool _computeHull(int vcount, ObjectArrayList<Vector3> vertices, PHullResult result, int vlimit) {
		List<int> trisCount = [];
		int ret = _calchull(vertices, vcount, result.indices, trisCount, vlimit);
		if (ret == 0) return false;
		result.indexCount = trisCount[0] * 3;
		result.faceCount = trisCount[0];
		result.vertices = vertices;
		result.vcount = vcount;
		return true;
	}

	Tri _allocateTriangle(int a, int b, int c) {
		Tri tr = Tri(a, b, c);
		tr.id = _tris.size;
		_tris.add(tr);

		return tr;
	}
	
	void _deAllocateTriangle(Tri? tri) {
    if(tri == null) return;
		assert (_tris.getQuick(tri.id) == tri);
		_tris.setQuick(tri.id, null);
	}
	
	void _b2bfix(Tri s, Tri t) {
		for (int i=0; i<3; i++) {
			int i1 = (i + 1) % 3;
			int i2 = (i + 2) % 3;
			int a = s.getCoord(i1);
			int b = s.getCoord(i2);
			assert (_tris.getQuick(s.neib(a, b))?.neib(b, a) == s.id);
			assert (_tris.getQuick(t.neib(a, b))?.neib(b, a) == t.id);
			_tris.getQuick(s.neib(a, b))?.setRef(t.neib(b, a));
			_tris.getQuick(t.neib(b, a))?.setRef(s.neib(a, b));
		}
	}

	void _removeb2b(Tri s, Tri t) {
		_b2bfix(s, t);
		_deAllocateTriangle(s);
		_deAllocateTriangle(t);
	}

	void _checkit(Tri t) {
		assert (_tris.getQuick(t.id) == t);
		for (int i=0; i<3; i++) {
			int i1 = (i + 1) % 3;
			int i2 = (i + 2) % 3;
			int a = t.getCoord(i1);
			int b = t.getCoord(i2);

			assert (a != b);
			assert (_tris.getQuick(t.n.getCoord(i))?.neib(b, a) == t.id);
		}
	}

	Tri? _extrudable(double epsilon) {
		Tri? t;
		for (int i=0; i<_tris.size; i++) {
			if (t == null || (_tris.getQuick(i) != null && t.rise < (_tris.getQuick(i)?.rise ?? 0))) {
				t = _tris.getQuick(i);
			}
		}
		return ((t?.rise ?? 0) > epsilon) ? t : null;
	}

	int _calchull(ObjectArrayList<Vector3> verts, int vertsCount, IntArrayList trisOut, List<int> trisCount, int vlimit) {
		int rc = _calchullgen(verts, vertsCount, vlimit);
		if (rc == 0) return 0;
		
		IntArrayList ts = IntArrayList();

		for (int i=0; i<_tris.size; i++) {
			if (_tris.getQuick(i) != null) {
				for (int j = 0; j < 3; j++) {
					ts.add((_tris.getQuick(i))?.getCoord(j) ?? 0);
				}
				_deAllocateTriangle(_tris.getQuick(i));
			}
		}
		trisCount[0] = ts.size() ~/ 3;
		MiscUtil.resizeArray(trisOut, ts.size(), 0);

		for (int i=0; i<ts.size(); i++) {
			trisOut.set(i, ts.get(i));
		}
		MiscUtil.resizeObjectArray(_tris, 0, null);

		return 1;
	}

	int _calchullgen(ObjectArrayList<Vector3> verts, int vertsCount, int vlimit) {
		if (vertsCount < 4) return 0;
		
		Vector3 tmp = Vector3.zero();
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		if (vlimit == 0) {
			vlimit = 1000000000;
		}
		//int j;
		Vector3 bmin = Vector3.copy(verts[0] ?? Vector3.zero());
		Vector3 bmax = Vector3.copy(verts[0] ?? Vector3.zero());
		IntArrayList isextreme = IntArrayList();
		//isextreme.reserve(vertsCount);
		IntArrayList allow = IntArrayList();
		//allow.reserve(vertsCount);

		for (int j=0; j<vertsCount; j++) {
			allow.add(1);
			isextreme.add(0);
			VectorUtil.setMin(bmin, verts.getQuick(j) ?? Vector3.zero());
			VectorUtil.setMax(bmax, verts.getQuick(j) ?? Vector3.zero());
		}
		tmp.sub2(bmax,bmin);
		double epsilon = tmp.length * 0.001;
		assert (epsilon != 0);

		Int4 p = _findSimplex(verts, vertsCount, allow, Int4());
		if (p.x == -1) {
			return 0; // simplex failed

		// a valid interior point
		}
		Vector3 center = Vector3.zero();
		VectorUtil.add(center, verts.getQuick(p.getCoord(0)) ?? Vector3.zero(), verts.getQuick(p.getCoord(1)), verts.getQuick(p.getCoord(2)), verts.getQuick(p.getCoord(3)));
		center.scale(1 / 4);

		Tri t0 = _allocateTriangle(p.getCoord(2), p.getCoord(3), p.getCoord(1));
		t0.n.set(2, 3, 1);
		Tri t1 = _allocateTriangle(p.getCoord(3), p.getCoord(2), p.getCoord(0));
		t1.n.set(3, 2, 0);
		Tri t2 = _allocateTriangle(p.getCoord(0), p.getCoord(1), p.getCoord(3));
		t2.n.set(0, 1, 3);
		Tri t3 = _allocateTriangle(p.getCoord(1), p.getCoord(0), p.getCoord(2));
		t3.n.set(1, 0, 2);
		isextreme.set(p.getCoord(0), 1);
		isextreme.set(p.getCoord(1), 1);
		isextreme.set(p.getCoord(2), 1);
		isextreme.set(p.getCoord(3), 1);
		_checkit(t0);
		_checkit(t1);
		_checkit(t2);
		_checkit(t3);

		Vector3 n = Vector3.zero();

		for (int j=0; j<_tris.size; j++) {
			Tri? t = _tris.getQuick(j);
			assert (t != null);
			assert (t!.vmax < 0);
			_triNormal(verts.getQuick(t!.getCoord(0)) ?? Vector3.zero(), verts.getQuick(t.getCoord(1)) ?? Vector3.zero(), verts.getQuick(t.getCoord(2)) ?? Vector3.zero(), n);
			t.vmax = _maxdirsterid(verts, vertsCount, n, allow);
			tmp.sub2(verts.getQuick(t.vmax),verts.getQuick(t.getCoord(0)));
			t.rise = n.dot(tmp);
		}
		Tri? te;
		vlimit -= 4;
		while (vlimit > 0 && ((te = _extrudable(epsilon)) != null)) {
			//Int3 ti = te;
			int v = te!.vmax;
			assert (v != -1);
			assert (isextreme.get(v) == 0);  // wtf we've already done this vertex
			isextreme.set(v, 1);
			//if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
			int j = _tris.size;
			while ((j--) != 0) {
				if (_tris.getQuick(j) == null) {
					continue;
				}
				Int3? t = _tris.getQuick(j);
				if (t != null && _above(verts, t, verts.getQuick(v) ?? Vector3.zero(), 0.01 * epsilon)) {
					_extrude(_tris.getQuick(j), v);
				}
			}
			// now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
			j = _tris.size;
			while ((j--) != 0) {
				if (_tris.getQuick(j) == null) {
					continue;
				}
				if (_tris.getQuick(j) != null && !_hasvert(_tris.getQuick(j)!, v)) {
					break;
				}
				Int3 nt = _tris.getQuick(j)!;
				tmp1.sub2(verts.getQuick(nt.getCoord(1)), verts.getQuick(nt.getCoord(0)));
				tmp2.sub2(verts.getQuick(nt.getCoord(2)), verts.getQuick(nt.getCoord(1)));
				tmp.cross2(tmp1, tmp2);
				if (_above(verts, nt, center, 0.01 * epsilon) || tmp.length < epsilon * epsilon * 0.1) {
					Tri? nb = _tris.getQuick(_tris.getQuick(j)!.n.getCoord(0));
					assert (nb != null);
					assert (!_hasvert(nb, v));
					assert ((nb?.id ?? 0) < j);
					_extrude(nb, v);
					j = _tris.size;
				}
			}
			j = _tris.size;
			while ((j--) != 0) {
				Tri? t = _tris[j];
				if (t == null) {
					continue;
				}
				if (t.vmax >= 0) {
					break;
				}
				_triNormal(verts[t.getCoord(0)] ?? Vector3.zero(), verts[t.getCoord(1)] ?? Vector3.zero(), verts[t.getCoord(2)] ?? Vector3.zero(), n);
				t.vmax = _maxdirsterid(verts, vertsCount, n, allow);
				if (isextreme.get(t.vmax) != 0) {
					t.vmax = -1; // already done that vertex - algorithm needs to be able to terminate.
				}
				else {
					tmp.sub2(verts[t.vmax],verts[t.getCoord(0)]);
					t.rise = n.dot(tmp);
				}
			}
			vlimit--;
		}
		return 1;
	}

	Int4 _findSimplex(ObjectArrayList<Vector3> verts, int vertsCount, IntArrayList allow, Int4 out) {
		Vector3 tmp = Vector3.zero();
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		List<Vector3> basis = [Vector3.zero(), Vector3.zero(), Vector3.zero()];
		basis[0].setValues(0.01, 0.02, 1.0);
		int p0 = _maxdirsterid(verts, vertsCount, basis[0], allow);
		tmp.negateFrom(basis[0]);
		int p1 = _maxdirsterid(verts, vertsCount, tmp, allow);
		basis[0].sub2(verts[p0],verts[p1]);
		if (p0 == p1 || (basis[0].x == 0 && basis[0].y == 0 && basis[0].z == 0)) {
			out.set(-1, -1, -1, -1);
			return out;
		}
		tmp.setValues(1, 0.02, 0);
		basis[1].cross2(tmp, basis[0]);
		tmp.setValues(-0.02, 1, 0);
		basis[2].cross2(tmp, basis[0]);
		if (basis[1].length > basis[2].length) {
			basis[1].normalize();
		}
		else {
			basis[1].setFrom(basis[2]);
			basis[1].normalize();
		}
		int p2 = _maxdirsterid(verts, vertsCount, basis[1], allow);
		if (p2 == p0 || p2 == p1) {
			tmp.negateFrom(basis[1]);
			p2 = _maxdirsterid(verts, vertsCount, tmp, allow);
		}
		if (p2 == p0 || p2 == p1) {
			out.set(-1, -1, -1, -1);
			return out;
		}
		basis[1].sub2(verts[p2],verts[p0]);
		basis[2].cross2(basis[1], basis[0]);
		basis[2].normalize();
		int p3 = _maxdirsterid(verts, vertsCount, basis[2], allow);
		if (p3 == p0 || p3 == p1 || p3 == p2) {
			tmp.negateFrom(basis[2]);
			p3 = _maxdirsterid(verts, vertsCount, tmp, allow);
		}
		if (p3 == p0 || p3 == p1 || p3 == p2) {
			out.set(-1, -1, -1, -1);
			return out;
		}
		assert (!(p0 == p1 || p0 == p2 || p0 == p3 || p1 == p2 || p1 == p3 || p2 == p3));

		tmp1.sub2(verts[p1],verts[p0]);
		tmp2.sub2(verts[p2],verts[p0]);
		tmp2.cross(tmp1);
		tmp1.sub2(verts[p3],verts[p0]);
		if (tmp1.dot(tmp2) < 0) {
			int swapTmp = p2;
			p2 = p3;
			p3 = swapTmp;
		}
		out.set(p0, p1, p2, p3);
		return out;
	}

	//private ConvexH convexHCrop(ConvexH convex,Plane slice);

	void _extrude(Tri? t0, int v) {
    if(t0 == null) return;
		Int3 t = Int3.fromInt3(t0);
		int n = _tris.size;
		Tri ta = _allocateTriangle(v, t.getCoord(1), t.getCoord(2));
		ta.n.set(t0.n.getCoord(0), n + 1, n + 2);
		_tris[t0.n.getCoord(0)]?.setRef(n + 0);//.neib(t.getCoord(1), t.getCoord(2))
		Tri tb = _allocateTriangle(v, t.getCoord(2), t.getCoord(0));
		tb.n.set(t0.n.getCoord(1), n + 2, n + 0);
		_tris[t0.n.getCoord(1)]?.setRef(n + 1);//.neib(t.getCoord(2), t.getCoord(0))
		Tri tc = _allocateTriangle(v, t.getCoord(0), t.getCoord(1));
		tc.n.set(t0.n.getCoord(2), n + 0, n + 1);
		_tris[t0.n.getCoord(2)]?.setRef(n + 2);//.neib(t.getCoord(0), t.getCoord(1))
		_checkit(ta);
		_checkit(tb);
		_checkit(tc);
		if (_hasvert(_tris[ta.n.getCoord(0)], v)) {
			_removeb2b(ta, _tris[ta.n.getCoord(0)]!);
		}
		if (_hasvert(_tris[tb.n.getCoord(0)], v)) {
			_removeb2b(tb, _tris[tb.n.getCoord(0)]!);
		}
		if (_hasvert(_tris[tc.n.getCoord(0)], v)) {
			_removeb2b(tc, _tris[tc.n.getCoord(0)]!);
		}
		_deAllocateTriangle(t0);
	}

	//private ConvexH test_cube();

	//BringOutYourDead (John Ratcliff): When you create a convex hull you hand it a large input set of vertices forming a 'point cloud'. 
	//After the hull is generated it give you back a set of polygon faces which index the *original* point cloud.
	//The thing is, often times, there are many 'dead vertices' in the point cloud that are on inter referenced by the hull.
	//The routine 'BringOutYourDead' find only the referenced vertices, copies them to an buffer, and re-indexes the hull so that it is a minimal representation.
	void _bringOutYourDead(ObjectArrayList<Vector3>? verts, int vcount, ObjectArrayList<Vector3> overts, List<int> ocount, IntArrayList indices, int indexcount) {
		IntArrayList tmpIndices = IntArrayList();
		for (int i=0; i<vertexIndexMapping.size(); i++) {
			tmpIndices.add(vertexIndexMapping.size());
		}

		IntArrayList usedIndices = IntArrayList();
		MiscUtil.resizeArray(usedIndices, vcount, 0);
		/*
		JAVA NOTE: redudant
		for (int i=0; i<vcount; i++) {
		usedIndices.set(i, 0);
		}
		*/

		ocount[0] = 0;

		for (int i=0; i<indexcount; i++) {
			int v = indices.get(i); // original array index

			assert (v >= 0 && v < vcount);

			if (usedIndices.get(v) != 0) { // if already remapped
				indices.set(i, usedIndices.get(v) - 1); // index to array
			}
			else {
				indices.set(i, ocount[0]);      // index mapping 
        if(verts?[v] != null){
				  overts.getQuick(ocount[0])?.setFrom(verts![v]!); // copy old vert to vert array
        }

				for (int k = 0; k < vertexIndexMapping.size(); k++) {
					if (tmpIndices.get(k) == v) {
						vertexIndexMapping.set(k, ocount[0]);
					}
				}

				ocount[0]++; // increment output vert count

				assert (ocount[0] >= 0 && ocount[0] <= vcount);

				usedIndices.set(v, ocount[0]); // assign index remapping
			}
		}
	}

	static const double _epsilon = 0.000001; /* close enough to consider two btScalaring point numbers to be 'the same'. */
	
	bool _cleanupVertices(
    int svcount,
    ObjectArrayList<Vector3>? svertices,
    int stride,
    List<int> vcount, // output number of vertices
    ObjectArrayList<Vector3> vertices, // location to store the results.
    double normalepsilon,
    Vector3? scale
  ) {
		
		if (svcount == 0) {
			return false;
		}

		vertexIndexMapping.clear();

		vcount[0] = 0;

		List<double> recip = List.filled(3, 0);

		if (scale != null) {
			scale.setValues(1, 1, 1);
		}

		List<double> bmin = [ double.infinity, double.infinity, double.infinity ];
		List<double> bmax = [ -double.infinity, -double.infinity, -double.infinity ];

		ObjectArrayList<Vector3>? vtxPtr = svertices;
		int vtxIdx = 0;

		//	if ( 1 )
		{
			for (int i=0; i < svcount; i++) {
				Vector3 p = vtxPtr![vtxIdx] ?? Vector3.zero();

				vtxIdx +=/*stride*/ 1;

				for (int j=0; j<3; j++) {
					if (VectorUtil.getCoord(p, j) < bmin[j]) {
						bmin[j] = VectorUtil.getCoord(p, j);
					}
					if (VectorUtil.getCoord(p, j) > bmax[j]) {
						bmax[j] = VectorUtil.getCoord(p, j);
					}
				}
			}
		}

		double dx = bmax[0] - bmin[0];
		double dy = bmax[1] - bmin[1];
		double dz = bmax[2] - bmin[2];

		Vector3 center = Vector3.zero();

		center.x = dx * 0.5 + bmin[0];
		center.y = dy * 0.5 + bmin[1];
		center.z = dz * 0.5 + bmin[2];

		if (dx < _epsilon || dy < _epsilon || dz < _epsilon || svcount < 3) {

			double len = double.infinity;

			if (dx > _epsilon && dx < len) len = dx;
			if (dy > _epsilon && dy < len) len = dy;
			if (dz > _epsilon && dz < len) len = dz;
			
			if (len == double.infinity) {
				dx = dy = dz = 0.01; // one centimeter
			}
			else {
				if (dx < _epsilon) dx = len * 0.05; // 1/5th the intest non-zero edge.
				if (dy < _epsilon) dy = len * 0.05;
				if (dz < _epsilon) dz = len * 0.05;
			}

			double x1 = center.x - dx;
			double x2 = center.x + dx;

			double y1 = center.y - dy;
			double y2 = center.y + dy;

			double z1 = center.z - dz;
			double z2 = center.z + dz;

			_addPoint(vcount, vertices, x1, y1, z1);
			_addPoint(vcount, vertices, x2, y1, z1);
			_addPoint(vcount, vertices, x2, y2, z1);
			_addPoint(vcount, vertices, x1, y2, z1);
			_addPoint(vcount, vertices, x1, y1, z2);
			_addPoint(vcount, vertices, x2, y1, z2);
			_addPoint(vcount, vertices, x2, y2, z2);
			_addPoint(vcount, vertices, x1, y2, z2);

			return true; // return cube
		}
		else {
			if (scale != null) {
				scale.x = dx;
				scale.y = dy;
				scale.z = dz;

				recip[0] = 1 / dx;
				recip[1] = 1 / dy;
				recip[2] = 1 / dz;

				center.x *= recip[0];
				center.y *= recip[1];
				center.z *= recip[2];
			}
		}

		vtxPtr = svertices;
		vtxIdx = 0;

		for (int i=0; i < svcount; i++) {
			Vector3 p = vtxPtr![vtxIdx] ?? Vector3.zero();
			vtxIdx +=/*stride*/ 1;

			double px = p.x;
			double py = p.y;
			double pz = p.z;

			if (scale != null) {
				px = px * recip[0]; // normalize
				py = py * recip[1]; // normalize
				pz = pz * recip[2]; // normalize
			}

			//		if ( 1 )
			{
				int j;

				for (j=0; j<vcount[0]; j++) {
					/// XXX might be broken
					Vector3 v = vertices[j] ?? Vector3.zero();

					double x = v.x;
					double y = v.y;
					double z = v.z;

					dx = (x - px).abs();
					dy = (y - py).abs();
					dz = (z - pz).abs();

					if (dx < normalepsilon && dy < normalepsilon && dz < normalepsilon) {
						// ok, it is close enough to the old one
						// now let us see if it is further from the center of the point cloud than the one we already recorded.
						// in which case we keep this one instead.

						double dist1 = _getDist(px, py, pz, center);
						double dist2 = _getDist(v.x, v.y, v.z, center);

						if (dist1 > dist2) {
							v.x = px;
							v.y = py;
							v.z = pz;
						}

						break;
					}
				}

				if (j == vcount[0]) {
					Vector3 dest = vertices[vcount[0]] ?? Vector3.zero();
					dest.x = px;
					dest.y = py;
					dest.z = pz;
					vcount[0]++;
				}

				vertexIndexMapping.add(j);
			}
		}

		// ok..now make sure we didn't prune so many vertices it is now invalid.
		//	if ( 1 )
		{
			bmin = [ double.infinity, double.infinity, double.infinity ];
			bmax = [ -double.infinity, -double.infinity, -double.infinity ];

			for (int i=0; i<vcount[0]; i++) {
				Vector3 p = vertices[i] ?? Vector3.zero();
				for (int j = 0; j < 3; j++) {
					if (VectorUtil.getCoord(p, j) < bmin[j]) {
						bmin[j] = VectorUtil.getCoord(p, j);
					}
					if (VectorUtil.getCoord(p, j) > bmax[j]) {
						bmax[j] = VectorUtil.getCoord(p, j);
					}
				}
			}

			dx = bmax[0] - bmin[0];
			dy = bmax[1] - bmin[1];
			dz = bmax[2] - bmin[2];

			if (dx < _epsilon || dy < _epsilon || dz < _epsilon || vcount[0] < 3) {
				double cx = dx * 0.5 + bmin[0];
				double cy = dy * 0.5 + bmin[1];
				double cz = dz * 0.5 + bmin[2];

				double len = double.infinity;

				if (dx >= _epsilon && dx < len) len = dx;
				if (dy >= _epsilon && dy < len) len = dy;
				if (dz >= _epsilon && dz < len) len = dz;
				
				if (len == double.infinity) {
					dx = dy = dz = 0.01; // one centimeter
				}
				else {
					if (dx < _epsilon) dx = len * 0.05; // 1/5th the intest non-zero edge.
					if (dy < _epsilon) dy = len * 0.05;
					if (dz < _epsilon) dz = len * 0.05;
				}

				double x1 = cx - dx;
				double x2 = cx + dx;

				double y1 = cy - dy;
				double y2 = cy + dy;

				double z1 = cz - dz;
				double z2 = cz + dz;

				vcount[0] = 0; // add box

				_addPoint(vcount, vertices, x1, y1, z1);
				_addPoint(vcount, vertices, x2, y1, z1);
				_addPoint(vcount, vertices, x2, y2, z1);
				_addPoint(vcount, vertices, x1, y2, z1);
				_addPoint(vcount, vertices, x1, y1, z2);
				_addPoint(vcount, vertices, x2, y1, z2);
				_addPoint(vcount, vertices, x2, y2, z2);
				_addPoint(vcount, vertices, x1, y2, z2);

				return true;
			}
		}

		return true;
	}

	////////////////////////////////////////////////////////////////////////////
	
	static bool _hasvert(Int3? t, int v) {
    if(t == null) return false;
		return (t.getCoord(0) == v || t.getCoord(1) == v || t.getCoord(2) == v);
	}
	
	static Vector3 _orth(Vector3 v, Vector3 out) {
		Vector3 a = Vector3.zero();
		a.setValues(0, 0, 1);
		a.cross2(v, a);

		Vector3 b = Vector3.zero();
		b.setValues(0, 1, 0);
		b.cross2(v, b);

		if (a.length > b.length) {
			out.normalizeFrom(a);
			return out;
		}
		else {
			out.normalizeFrom(b);
			return out;
		}
	}
	
	static int _maxdirfiltered(ObjectArrayList<Vector3> p, int count, Vector3 dir, IntArrayList allow) {
		assert (count != 0);
		int m = -1;
		for (int i=0; i<count; i++) {
			if (allow.get(i) != 0) {
				if (m == -1 || (p[i]?.dot(dir) ?? 0) > (p[m]?.dot(dir) ?? 0)) {
					m = i;
				}
			}
		}
		assert (m != -1);
		return m;
	}
	
	static int _maxdirsterid(ObjectArrayList<Vector3> p, int count, Vector3 dir, IntArrayList allow) {
		Vector3 tmp = Vector3.zero();
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();
		Vector3 u = Vector3.zero();
		Vector3 v = Vector3.zero();

		int m = -1;
		while (m == -1) {
			m = _maxdirfiltered(p, count, dir, allow);
			if (allow.get(m) == 3) {
				return m;
			}
			_orth(dir, u);
			v.cross2(u, dir);
			int ma = -1;
			for (double x = 0; x <= 360; x += 45) {
				double s = sin(BulletGlobals.simdRadsPerDeg * (x));
				double c = cos(BulletGlobals.simdRadsPerDeg * (x));

				tmp1.scaleFrom(s, u);
				tmp2.scaleFrom(c, v);
				tmp.add2(tmp1,tmp2);
				tmp.scale(0.025);
				tmp.add(dir);
				int mb = _maxdirfiltered(p, count, tmp, allow);
				if (ma == m && mb == m) {
					allow.set(m, 3);
					return m;
				}
				if (ma != -1 && ma != mb) { // Yuck - this is really ugly
					int mc = ma;
					for (double xx = x - 40; xx <= x; xx += 5) {
						s = sin(BulletGlobals.simdRadsPerDeg * (xx));
						c = cos(BulletGlobals.simdRadsPerDeg * (xx));

						tmp1.scaleFrom(s, u);
						tmp2.scaleFrom(c, v);
						tmp.add2(tmp1,tmp2);
						tmp.scale(0.025);
						tmp.add(dir);

						int md = _maxdirfiltered(p, count, tmp, allow);
						if (mc == m && md == m) {
							allow.set(m, 3);
							return m;
						}
						mc = md;
					}
				}
				ma = mb;
			}
			allow.set(m, 0);
			m = -1;
		}
		assert (false);
		return m;
	}
	
	static Vector3 _triNormal(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 out) {
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		// return the normal of the triangle
		// inscribed by v0, v1, and v2
		tmp1.sub2(v1,v0);
		tmp2.sub2(v2,v1);
		Vector3 cp = Vector3.zero();
		cp.cross2(tmp1, tmp2);
		double m = cp.length;
		if (m == 0) {
			out.setValues(1, 0,0);
			return out;
		}
		out.scaleFrom(1/ m, cp);
		return out;
	}
	
	static bool _above(ObjectArrayList<Vector3> vertices, Int3 t, Vector3 p, double epsilon) {
		Vector3 n = _triNormal(vertices[t.getCoord(0)] ?? Vector3.zero(), vertices[t.getCoord(1)] ?? Vector3.zero(), vertices[t.getCoord(2)] ?? Vector3.zero(), Vector3.zero());
		Vector3 tmp = Vector3.zero();
		tmp.sub2(p,vertices[t.getCoord(0)]);
		return (n.dot(tmp) > epsilon); // _epsilon???
	}
	
	static void _releaseHull(PHullResult result) {
		if (result.indices.isNotEmpty) {
			result.indices.clear();
		}

		result.vcount = 0;
		result.indexCount = 0;
		result.vertices = null;
	}
	
	static void _addPoint(List<int> vcount, ObjectArrayList<Vector3> p, double x, double y, double z) {
		// XXX, might be broken
		Vector3 dest = p[vcount[0]] ?? Vector3.zero();
		dest.x = x;
		dest.y = y;
		dest.z = z;
		vcount[0]++;
	}
	
	static double _getDist(double px, double py, double pz, Vector3 p2) {
		double dx = px - p2.x;
		double dy = py - p2.y;
		double dz = pz - p2.z;

		return dx*dx + dy*dy + dz*dz;
	}
	
}
