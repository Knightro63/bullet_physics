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

import "package:bullet_physics/linearmath/quaternion_util.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';


enum ResultsStatus {
  separated,		/* Shapes doesnt penetrate												*/ 
  penetrating,	/* Shapes are penetrating												*/ 
  gjkFailed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/ 
  epaFailed,		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/ 
}

class _gjkValues{
	static const double cstInf = BulletGlobals.simdInfinity;
	//static const double cstPi = BulletGlobals.simdPi;
	static const double cst2Pi = BulletGlobals.simd2pi;
	static const int gjkMaxiterations = 128;
	static const int gjkHashsize = 1 << 6;
	static const int gjkHashmask = gjkHashsize - 1;
	static const double gjkInsimplexEPS = 0.0001;
	static const double gjkSqinsimplexEPS = gjkInsimplexEPS * gjkInsimplexEPS;
	static const int epaMaxiterations = 256;
	static const double epaInfaceEPS = 0.01;
	static const double epaAccuracy = 0.001;
	
	////////////////////////////////////////////////////////////////////////////

	static List<int> mod3 = [ 0, 1, 2, 0, 1 ];

	static final List<List<int>> tetrahedronFidx/*[4][3]*/ = [[2,1,0],[3,0,1],[3,1,2],[3,2,0]];
	static final List<List<int>> tetrahedronEidx/*[6][4]*/ = [[0,0,2,1],[0,1,1,1],[0,2,3,1],[1,0,3,2],[2,0,1,2],[3,0,2,2]];

	static final List<List<int>> hexahedronFidx/*[6][3]*/ = [[2,0,4],[4,1,2],[1,4,0],[0,3,1],[0,2,3],[1,3,2]];
	static final List<List<int>> hexahedronEidx/*[9][4]*/ = [[0,0,4,0],[0,1,2,1],[0,2,1,2],[1,1,5,2],[1,0,2,0],[2,2,3,2],[3,1,5,0],[3,0,4,2],[5,1,4,1]];
}

class GjkEpaSolver {
	//final ArrayPool<List<double>> doubleArrays = ArrayPool.get(double.class);	
	final GJK _gjk = GJK();
	
	bool collide(
    ConvexShape? shape0, 
    Transform wtrs0,
    ConvexShape? shape1, 
    Transform wtrs1,
    double radialmargin,
    Results results
  ) {
		
		// Initialize
		results.witnesses[0].setValues(0, 0, 0);
		results.witnesses[1].setValues(0, 0, 0);
		results.normal.setValues(0, 0, 0);
		results.depth = 0;
		results.status = ResultsStatus.separated;
		results.epaIterations = 0;
		results.gjkIterations = 0;
		/* Use GJK to locate origin		*/
		_gjk.init(
				wtrs0.basis, wtrs0.origin, shape0,
				wtrs1.basis, wtrs1.origin, shape1,
				radialmargin + _gjkValues.epaAccuracy);
		try {
			bool collide = _gjk.searchOrigin();
			results.gjkIterations = _gjk.iterations + 1;
			if (collide) {
				/* Then EPA for penetration depth	*/
				EPA epa = EPA(_gjk);
				double pd = epa.evaluatePD();
				results.epaIterations = epa.iterations + 1;
				if (pd > 0) {
					results.status = ResultsStatus.penetrating;
					results.normal.setFrom(epa.normal);
					results.depth = pd;
					results.witnesses[0].setFrom(epa.nearest[0]);
					results.witnesses[1].setFrom(epa.nearest[1]);
					return (true);
				}
				else {
					if (epa.failed) {
						results.status = ResultsStatus.epaFailed;
					}
				}
			}
			else {
				if (_gjk.failed) {
					results.status = ResultsStatus.gjkFailed;
				}
			}
			return (false);
		}
		finally {
			_gjk.destroy();
		}
	}
	
}
////////////////////////////////////////////////////////////////////////////

class GJK {
  //final BulletStack stack = BulletStack.get();
  final List<He?> table = List.filled(_gjkValues.gjkHashsize, null);
  final List<Matrix3> wrotations/*[2]*/ = [ Matrix3.zero(), Matrix3.zero() ];
  final List<Vector3> positions/*[2]*/ = [ Vector3.zero(), Vector3.zero() ];
  final List<ConvexShape?> shapes = [null,null];//ConvexShape[2];
  final List<Mkv> simplex = [Mkv(),Mkv(),Mkv(),Mkv(),Mkv()];//Mkv[5];
  final Vector3 ray = Vector3.zero();
  /*unsigned*/ int order = 0;
  /*unsigned*/ int iterations = 0;
  double margin = 0;
  bool failed = false;

  GJK([
    Matrix3? wrot0, 
    Vector3? pos0, 
    ConvexShape? shape0,
    Matrix3? wrot1, 
    Vector3? pos1, 
    ConvexShape? shape1,
    double pmargin = 0
  ]) {
    init(wrot0, pos0, shape0, wrot1, pos1, shape1, pmargin);
  }
  
  void init(
    Matrix3? wrot0, 
    Vector3? pos0, 
    ConvexShape? shape0,
    Matrix3? wrot1, 
    Vector3? pos1, 
    ConvexShape? shape1,
    double pmargin
  ) {
    if(wrot0 != null){
      wrotations[0].setFrom(wrot0);
    }
    if(pos0 != null){
      positions[0].setFrom(pos0);
    }
    shapes[0] = shape0;
    if(wrot1 != null){
      wrotations[1].setFrom(wrot1);
    }
    if(pos1 != null){
      positions[1].setFrom(pos1);
    }
    shapes[1] = shape1;
    //sa		=psa;
    //sablock	=sa->beginBlock();
    margin = pmargin;
  }
  
  void destroy() {}
  
  // vdh: very dummy hash
  /*unsigned*/ int hash(Vector3 v) {
    int h = (v.x * 15461).toInt() ^ (v.y * 83003).toInt() ^ (v.z * 15473).toInt();
    return (h * 169639) & _gjkValues.gjkHashmask;
  }

  Vector3 localSupport(Vector3 d, /*unsigned*/ int i, Vector3 out) {
    Vector3 tmp = Vector3.zero();
    MatrixUtil.transposeTransform(tmp, d, wrotations[i]);

    shapes[i]?.localGetSupportingVertex(tmp, out);
    wrotations[i].transform(out);
    out.add(positions[i]);

    return out;
  }
  
  void support(Vector3 d, Mkv v) {
    v.r.setFrom(d);

    Vector3 tmp1 = localSupport(d, 0, Vector3.zero());

    Vector3 tmp = Vector3.zero();
    tmp.setFrom(d);
    tmp.negate();
    Vector3 tmp2 = localSupport(tmp, 1, Vector3.zero());

    v.w.sub2(tmp1, tmp2);
    v.w.scaleAdd(margin, d, v.w);
  }

  bool fetchSupport() {
    int h = hash(ray);
    He? e = table[h];
    while (e != null) {
      if (e.v.equals(ray)) {
        --order;
        return false;
      }
      else {
        e = e.n;
      }
    }
    //e = (He*)sa->allocate(sizeof(He));
    e = He();
    e.v.setFrom(ray);
    e.n = table[h];
    table[h] = e;
    support(ray, simplex[++order]);
    return (ray.dot(simplex[order].w) > 0);
  }

  bool solveSimplex2(Vector3 ao, Vector3 ab) {
    if (ab.dot(ao) >= 0) {
      Vector3 cabo = Vector3.zero();
      cabo.cross2(ab, ao);
      if (cabo.length2 > _gjkValues.gjkSqinsimplexEPS) {
        ray.cross2(cabo, ab);
      }
      else {
        return true;
      }
    }
    else {
      order = 0;
      simplex[0].set(simplex[1]);
      ray.setFrom(ao);
    }
    return (false);
  }

  bool solveSimplex3(Vector3 ao, Vector3 ab, Vector3 ac){
    Vector3 tmp = Vector3.zero();
    tmp.cross2(ab, ac);
    return (solveSimplex3a(ao,ab,ac,tmp));
  }
  
  bool solveSimplex3a(Vector3 ao, Vector3 ab, Vector3 ac, Vector3 cabc) {
    // TODO: optimize

    Vector3 tmp = Vector3.zero();
    tmp.cross2(cabc, ab);

    Vector3 tmp2 = Vector3.zero();
    tmp2.cross2(cabc, ac);

    if (tmp.dot(ao) < -_gjkValues.gjkInsimplexEPS) {
      order = 1;
      simplex[0].set(simplex[1]);
      simplex[1].set(simplex[2]);
      return solveSimplex2(ao, ab);
    }
    else if (tmp2.dot(ao) > _gjkValues.gjkInsimplexEPS) {
      order = 1;
      simplex[1].set(simplex[2]);
      return solveSimplex2(ao, ac);
    }
    else {
      double d = cabc.dot(ao);
      if (d.abs() > _gjkValues.gjkInsimplexEPS) {
        if (d > 0) {
          ray.setFrom(cabc);
        }
        else {
          ray.negateFrom(cabc);

          Mkv swapTmp = Mkv();
          swapTmp.set(simplex[0]);
          simplex[0].set(simplex[1]);
          simplex[1].set(swapTmp);
        }
        return false;
      }
      else {
        return true;
      }
    }
  }
  
  bool solveSimplex4(Vector3 ao, Vector3 ab, Vector3 ac, Vector3 ad) {
    // TODO: optimize

    Vector3 crs = Vector3.zero();

    Vector3 tmp = Vector3.zero();
    tmp.cross2(ab, ac);

    Vector3 tmp2 = Vector3.zero();
    tmp2.cross2(ac, ad);

    Vector3 tmp3 = Vector3.zero();
    tmp3.cross2(ad, ab);

    if (tmp.dot(ao) > _gjkValues.gjkInsimplexEPS) {
      crs.setFrom(tmp);
      order = 2;
      simplex[0].set(simplex[1]);
      simplex[1].set(simplex[2]);
      simplex[2].set(simplex[3]);
      return solveSimplex3a(ao, ab, ac, crs);
    }
    else if (tmp2.dot(ao) > _gjkValues.gjkInsimplexEPS) {
      crs.setFrom(tmp2);
      order = 2;
      simplex[2].set(simplex[3]);
      return solveSimplex3a(ao, ac, ad, crs);
    }
    else if (tmp3.dot(ao) > _gjkValues.gjkInsimplexEPS) {
      crs.setFrom(tmp3);
      order = 2;
      simplex[1].set(simplex[0]);
      simplex[0].set(simplex[2]);
      simplex[2].set(simplex[3]);
      return solveSimplex3a(ao, ad, ab, crs);
    }
    else {
      return (true);
    }
  }
  
  bool searchOrigin([Vector3? initray]) {
    initray ??= Vector3(1,0,0);
    Vector3 tmp1 = Vector3.zero();
    Vector3 tmp2 = Vector3.zero();
    Vector3 tmp3 = Vector3.zero();
    Vector3 tmp4 = Vector3.zero();

    iterations = 0;
    order = -1;
    failed = false;
    ray.setFrom(initray);
    ray.normalize();

    //Arrays.fill(table, null);

    fetchSupport();
    ray.negateFrom(simplex[0].w);
    for (; iterations < _gjkValues.gjkMaxiterations; iterations++) {
      double rl = ray.length;
      ray.scale(1 / (rl > 0 ? rl : 1));
      if (fetchSupport()) {
        bool found = false;
        switch (order) {
          case 1: {
            tmp1.negateFrom(simplex[1].w);
            tmp2.sub2(simplex[0].w, simplex[1].w);
            found = solveSimplex2(tmp1, tmp2);
            break;
          }
          case 2: {
            tmp1.negateFrom(simplex[2].w);
            tmp2.sub2(simplex[1].w, simplex[2].w);
            tmp3.sub2(simplex[0].w, simplex[2].w);
            found = solveSimplex3(tmp1, tmp2, tmp3);
            break;
          }
          case 3: {
            tmp1.negateFrom(simplex[3].w);
            tmp2.sub2(simplex[2].w, simplex[3].w);
            tmp3.sub2(simplex[1].w, simplex[3].w);
            tmp4.sub2(simplex[0].w, simplex[3].w);
            found = solveSimplex4(tmp1, tmp2, tmp3, tmp4);
            break;
          }
        }
        if (found) {
          return true;
        }
      }
      else {
        return false;
      }
    }
    failed = true;
    return false;
  }
  
  bool encloseOrigin() {
    Vector3 tmp = Vector3.zero();
    Vector3 tmp1 = Vector3.zero();
    Vector3 tmp2 = Vector3.zero();

    switch (order) {
      // Point
      case 0:
        break;
      // Line
      case 1: {
        Vector3 ab = Vector3.zero();
        ab.sub2(simplex[1].w, simplex[0].w);

        List<Vector3> b = [ Vector3.zero(), Vector3.zero(), Vector3.zero() ];
        b[0].setValues(1, 0, 0);
        b[1].setValues(0, 1, 0);
        b[2].setValues(0, 0, 1);
        
        b[0].cross2(ab, b[0]);
        b[1].cross2(ab, b[1]);
        b[2].cross2(ab, b[2]);

        List<double> m = [ b[0].length2, b[1].length2, b[2].length2];

        Quaternion tmpQuat = Quaternion(0,0,0,1);
        tmp.normalizeFrom(ab);
        QuaternionUtil.setRotation(tmpQuat, tmp, _gjkValues.cst2Pi / 3);

        Matrix3 r = Matrix3.zero();
        MatrixUtil.setRotation(r, tmpQuat);

        Vector3 w = Vector3.zero();
        w.setFrom(b[m[0] > m[1] ? m[0] > m[2] ? 0 : 2 : m[1] > m[2] ? 1 : 2]);

        tmp.normalizeFrom(w);
        support(tmp, simplex[4]); r.transform(w);
        tmp.normalizeFrom(w);
        support(tmp, simplex[2]); r.transform(w);
        tmp.normalizeFrom(w);
        support(tmp, simplex[3]); r.transform(w);
        order = 4;
        return (true);
      }
      // Triangle
      case 2: {
        tmp1.sub2(simplex[1].w, simplex[0].w);
        tmp2.sub2(simplex[2].w, simplex[0].w);
        Vector3 n = Vector3.zero();
        n.cross2(tmp1, tmp2);
        n.normalize();

        support(n, simplex[3]);

        tmp.negateFrom(n);
        support(tmp, simplex[4]);
        order = 4;
        return (true);
      }
      // Tetrahedron
      case 3:
        return (true);
      // Hexahedron
      case 4:
        return (true);
    }
    return (false);
  }
  
}
class Mkv {
  final Vector3 w = Vector3.zero(); // Minkowski vertice
  final Vector3 r = Vector3.zero(); // Ray

  void set(Mkv m) {
    w.setFrom(m.w);
    r.setFrom(m.r);
  }
}

class He {
  final Vector3 v = Vector3.zero();
  He? n;
}
class Results {
  ResultsStatus? status;
  final List<Vector3> witnesses/*[2]*/ = [ Vector3.zero(), Vector3.zero() ];
  final Vector3 normal = Vector3.zero();
  double depth = 0;
  int epaIterations = 0;
  int gjkIterations = 0;
}
class Face{
  final List<Mkv?> v = [null,null,null];
  final List<Face?> f = [null,null,null];
  final List<int> e = [0,0,0];
  final Vector3 n = Vector3.zero();
  double d = 0;
  int mark = 0;
  Face? prev;
  Face? next;
}

class EPA {
  //final BulletStack stack = BulletStack.get();
  
  late GJK gjk;
  //btStackAlloc* sa;
  Face? root;
  int nfaces = 0;
  int iterations = 0;
  final List<List<Vector3>> features = [[],[]];//Vector3[2][3];
  final List<Vector3> nearest = [ Vector3.zero(), Vector3.zero() ];
  final Vector3 normal = Vector3.zero();
  double depth = 0;
  bool failed = false;

  EPA(GJK pgjk) {
    gjk = pgjk;
    for (int i=0; i<features.length; i++) {
      for (int j=0; j<features[i].length; j++) {
        features[i].add(Vector3.zero());
      }
    }
  }
  
  Vector3 getCoordinates(Face face, Vector3 out) {
    Vector3 tmp = Vector3.zero();
    Vector3 tmp1 = Vector3.zero();
    Vector3 tmp2 = Vector3.zero();

    Vector3 o = Vector3.zero();
    o.scaleFrom(-face.d, face.n);

    List<double> a = [0,0,0];//doubleArrays.getFixed(3);

    tmp1.sub2(face.v[0]?.w, o);
    tmp2.sub2(face.v[1]?.w, o);
    tmp.cross2(tmp1, tmp2);
    a[0] = tmp.length;

    tmp1.sub2(face.v[1]?.w, o);
    tmp2.sub2(face.v[2]?.w, o);
    tmp.cross2(tmp1, tmp2);
    a[1] = tmp.length;

    tmp1.sub2(face.v[2]?.w, o);
    tmp2.sub2(face.v[0]?.w, o);
    tmp.cross2(tmp1, tmp2);
    a[2] = tmp.length;

    double sm = a[0] + a[1] + a[2];

    out.setValues(a[1], a[2], a[0]);
    out.scale(1 / (sm > 0 ? sm : 1));

    //doubleArrays.release(a);

    return out;
  }
  
  Face? findBest() {
    Face? bf;
    if (root != null) {
      Face? cf = root;
      double bd = _gjkValues.cstInf;
      do {
        if ((cf?.d ?? 0) < bd) {
          bd = cf?.d ?? 0;
          bf = cf;
        }
      }
      while (null != (cf = cf?.next));
    }
    return bf;
  }

  bool set(Face? f, Mkv? a, Mkv? b, Mkv? c) {
    Vector3 tmp1 = Vector3.zero();
    Vector3 tmp2 = Vector3.zero();
    Vector3 tmp3 = Vector3.zero();

    Vector3 nrm = Vector3.zero();
    tmp1.sub2(b?.w, a?.w);
    tmp2.sub2(c?.w, a?.w);
    nrm.cross2(tmp1, tmp2);

    double len = nrm.length;

    tmp1.cross2(a?.w, b?.w);
    tmp2.cross2(b?.w, c?.w);
    tmp3.cross2(c?.w, a?.w);

    bool valid = (tmp1.dot(nrm) >= -_gjkValues.epaInfaceEPS) &&
        (tmp2.dot(nrm) >= -_gjkValues.epaInfaceEPS) &&
        (tmp3.dot(nrm) >= -_gjkValues.epaInfaceEPS);

    f?.v[0] = a;
    f?.v[1] = b;
    f?.v[2] = c;
    f?.mark = 0;
    f?.n.scaleFrom(1 / (len > 0 ? len : _gjkValues.cstInf), nrm);
    f?.d = max(0, -f.n.dot(a?.w ?? Vector3.zero()));
    return valid;
  }
  
  Face newFace(Mkv? a, Mkv? b, Mkv? c) {
    Face pf = Face();
    if (set(pf, a, b, c)) {
      if (root != null) {
        root?.prev = pf;
      }
      pf.prev = null;
      pf.next = root;
      root = pf;
      ++nfaces;
    }
    else {
      pf.prev = pf.next = null;
    }
    return (pf);
  }

  void detach(Face? face) {
    if (face?.prev != null || face?.next != null) {
      --nfaces;
      if (face == root) {
        root = face?.next;
        root?.prev = null;
      }
      else {
        if (face?.next == null) {
          face?.prev?.next = null;
        }
        else {
          face?.prev?.next = face.next;
          face?.next?.prev = face.prev;
        }
      }
      face?.prev = face.next = null;
    }
  }

  void link(Face? f0, int e0, Face? f1, int e1) {
    f0?.f[e0] = f1; 
    f1?.e[e1] = e0;
    f1?.f[e1] = f0; 
    f0?.e[e0] = e1;
  }

  Mkv support(Vector3 w) {
    Mkv v = Mkv();
    gjk.support(w, v);
    return v;
  }
  
  int buildHorizon(int markid, Mkv w, Face? f, int e, List<Face?> cf, List<Face?> ff) {
    int ne = 0;
    if (f?.mark != markid) {
      int e1 = _gjkValues.mod3[e + 1];
      if (((f?.n.dot(w.w) ?? 0) + (f?.d ?? 0)) > 0) {
        Face nf = newFace(f?.v[e1], f?.v[e], w);
        link(nf, 0, f, e);
        if (cf[0] != null) {
          link(cf[0], 1, nf, 2);
        }
        else {
          ff[0] = nf;
        }
        cf[0] = nf;
        ne = 1;
      }
      else {
        int e2 = _gjkValues.mod3[e + 2];
        detach(f);
        f?.mark = markid;
        ne += buildHorizon(markid, w, f?.f[e1], f?.e[e1] ?? 0, cf, ff);
        ne += buildHorizon(markid, w, f?.f[e2], f?.e[e2] ?? 0, cf, ff);
      }
    }
    return (ne);
  }
  
  double evaluatePD([double accuracy = _gjkValues.epaAccuracy]) {
    try {
      Vector3 tmp = Vector3.zero();

      //btBlock* sablock = sa->beginBlock();
      Face? bestface;
      int markid = 1;
      depth = -_gjkValues.cstInf;
      normal.setValues(0, 0, 0);
      root = null;
      nfaces = 0;
      iterations = 0;
      failed = false;
      /* Prepare hull		*/
      if (gjk.encloseOrigin()) {
        //const U* pfidx = 0;
        List<List<int>>? pfidxPtr;
        int pfidxIndex = 0;

        int nfidx = 0;
        //const U* peidx = 0;
        List<List<int>>? peidxPtr;
        int peidxIndex = 0;

        int neidx = 0;
        List<Mkv?> basemkv = [null,null,null,null,null];// Mkv[5];
        List<Face?> basefaces = [null,null,null,null,null,null];//Face[6];
        switch (gjk.order) {
          // Tetrahedron
          case 3:
              {
              //pfidx=(const U*)fidx;
              pfidxPtr = _gjkValues.tetrahedronFidx;
              pfidxIndex = 0;

              nfidx = 4;

              //peidx=(const U*)eidx;
              peidxPtr = _gjkValues.tetrahedronEidx;
              peidxIndex = 0;

              neidx = 6;
            }
            break;
          // Hexahedron
          case 4:
              {
              //pfidx=(const U*)fidx;
              pfidxPtr = _gjkValues.hexahedronFidx;
              pfidxIndex = 0;

              nfidx = 6;

              //peidx=(const U*)eidx;
              peidxPtr = _gjkValues.hexahedronEidx;
              peidxIndex = 0;

              neidx = 9;
            }
            break;
        }

        for (int i = 0; i <= gjk.order; ++i) {
          basemkv[i] = Mkv();
          basemkv[i]?.set(gjk.simplex[i]);
        }
        for (int i = 0; i < nfidx; ++i, pfidxIndex++) {
          basefaces[i] = newFace(basemkv[pfidxPtr?[pfidxIndex][0] ?? 0], basemkv[pfidxPtr?[pfidxIndex][1] ?? 0], basemkv[pfidxPtr?[pfidxIndex][2] ?? 0]);
        }
        for (int i = 0; i < neidx; ++i, peidxIndex++) {
          link(basefaces[peidxPtr?[peidxIndex][0] ?? 0], peidxPtr?[peidxIndex][1] ?? 0, basefaces[peidxPtr?[peidxIndex][2]?? 0] , peidxPtr?[peidxIndex][3] ?? 0);
        }
      }
      if (0 == nfaces) {
        //sa->endBlock(sablock);
        return (depth);
      }
      /* Expand hull		*/
      for (; iterations < _gjkValues.epaMaxiterations; ++iterations) {
        Face? bf = findBest();
        if (bf != null) {
          tmp.negateFrom(bf.n);
          Mkv w = support(tmp);
          double d = bf.n.dot(w.w) + bf.d;
          bestface = bf;
          if (d < -accuracy) {
            List<Face?> cf = [null];
            List<Face?> ff = [null];
            int nf = 0;
            detach(bf);
            bf.mark = ++markid;
            for (int i = 0; i < 3; ++i) {
              nf += buildHorizon(markid, w, bf.f[i], bf.e[i], cf, ff);
            }
            if (nf <= 2) {
              break;
            }
            link(cf[0], 1, ff[0], 2);
          }
          else {
            break;
          }
        }
        else {
          break;
        }
      }
      /* Extract contact	*/
      if (bestface != null) {
        Vector3 b = getCoordinates(bestface, Vector3.zero());
        normal.setFrom(bestface.n);
        depth = max(0, bestface.d);
        for (int i = 0; i < 2; ++i) {
          double s = i != 0 ? -1 : 1;
          for (int j = 0; j < 3; ++j) {
            tmp.scaleFrom(s, bestface.v[j]?.r);
            gjk.localSupport(tmp, i, features[i][j]);
          }
        }

        Vector3 tmp1 = Vector3.zero();
        Vector3 tmp2 = Vector3.zero();
        Vector3 tmp3 = Vector3.zero();

        tmp1.scaleFrom(b.x, features[0][0]);
        tmp2.scaleFrom(b.y, features[0][1]);
        tmp3.scaleFrom(b.z, features[0][2]);
        VectorUtil.add(nearest[0], tmp1, tmp2, tmp3);

        tmp1.scaleFrom(b.x, features[1][0]);
        tmp2.scaleFrom(b.y, features[1][1]);
        tmp3.scaleFrom(b.z, features[1][2]);
        VectorUtil.add(nearest[1], tmp1, tmp2, tmp3);
      }
      else {
        failed = true;
      }
      //sa->endBlock(sablock);
      return (depth);
    }
    finally {
    }
  }
  
}