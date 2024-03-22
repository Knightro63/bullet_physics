import 'package:bullet_physics/core/bullet_globals.dart';
import 'package:bullet_physics/extras/gimpact/box_collision.dart';
import 'package:bullet_physics/extras/gimpact/plane_intersection_type.dart';
import 'package:bullet_physics/linearmath/matrix_util.dart';
import 'package:bullet_physics/linearmath/transform.dart';
import 'package:bullet_physics/linearmath/vector_util.dart';
import 'package:vector_math/vector_math.dart';
import 'dart:math' as Math;

////////////////////////////////////////////////////////////////////////////
class AABB {
  final Vector3 min = Vector3.zero();
  final Vector3 max = Vector3.zero();

  AABB([Vector3? v1, Vector3? v2, Vector3? v3, double? margin]) {
    if(margin != null && v1 != null && v2 != null && v3 != null){
      calcFromTriangleMargin(v1, v2, v3, margin);
    }
    else if(v1 != null && v2 != null && v3 != null){
      calcFromTriangle(v1, v2, v3);
    }
  }

  AABB.fromAABB(AABB other, [double margin = 0]) {
    set(other);
    min.x -= margin;
    min.y -= margin;
    min.z -= margin;
    max.x += margin;
    max.y += margin;
    max.z += margin;
  }

	static bool greater(double x, double y) {
		return (x).abs() > y;
	}

	static double max3(double a, double b, double c) {
		return Math.max(a, Math.max(b, c));
	}

	static double min3(double a, double b, double c) {
		return Math.min(a, Math.min(b, c));
	}

  void init(Vector3 v1, Vector3 v2, Vector3 v3, double margin) {
    calcFromTriangleMargin(v1, v2, v3, margin);
  }

  void set(AABB other) {
    min.setFrom(other.min);
    max.setFrom(other.max);
  }

  void invalidate() {
    min.setValues(BulletGlobals.simdInfinity, BulletGlobals.simdInfinity, BulletGlobals.simdInfinity);
    max.setValues(-BulletGlobals.simdInfinity, -BulletGlobals.simdInfinity, -BulletGlobals.simdInfinity);
  }

  void incrementMargin(double margin) {
    min.x -= margin;
    min.y -= margin;
    min.z -= margin;
    max.x += margin;
    max.y += margin;
    max.z += margin;
  }

  void copyWithMargin(AABB other, double margin) {
    min.x = other.min.x - margin;
    min.y = other.min.y - margin;
    min.z = other.min.z - margin;

    max.x = other.max.x + margin;
    max.y = other.max.y + margin;
    max.z = other.max.z + margin;
  }
  
  void calcFromTriangle(Vector3 v1, Vector3 v2, Vector3 v3) {
    min.x = min3(v1.x, v2.x, v3.x);
    min.y = min3(v1.y, v2.y, v3.y);
    min.z = min3(v1.z, v2.z, v3.z);

    max.x = min3(v1.x, v2.x, v3.x);
    max.y = min3(v1.y, v2.y, v3.y);
    max.z = min3(v1.z, v2.z, v3.z);
  }

  void calcFromTriangleMargin(Vector3 v1, Vector3 v2, Vector3 v3, double margin) {
    calcFromTriangle(v1, v2, v3);
    min.x -= margin;
    min.y -= margin;
    min.z -= margin;
    max.x += margin;
    max.y += margin;
    max.z += margin;
  }
  
  /**
   * Apply a transform to an AABB.
   */
  void appyTransform(Transform? trans) {
    if(trans == null) return;
    Vector3 tmp = Vector3.zero();

    Vector3 center = Vector3.zero();
    center.add2(max,min);
    center.scale(0.5);

    Vector3 extends_ = Vector3.zero();
    extends_.sub2(max,center);

    // Compute center
    trans.transform(center);

    Vector3 textends = Vector3.zero();

    trans.basis.getRowWith(0, tmp);
    tmp.absolute();
    textends.x = extends_.dot(tmp);

    trans.basis.getRowWith(1, tmp);
    tmp.absolute();
    textends.y = extends_.dot(tmp);

    trans.basis.getRowWith(2, tmp);
    tmp.absolute();
    textends.z = extends_.dot(tmp);

    min.sub2(center,textends);
    max.add2(center,textends);
  }

  /**
   * Apply a transform to an AABB.
   */
  void appyTransformTransCache(BoxBoxTransformCache trans) {
    Vector3 tmp = Vector3.zero();

    Vector3 center = Vector3.zero();
    center.add2(max,min);
    center.scale(0.5);

    Vector3 extends_ = Vector3.zero();
    extends_.sub2(max,center);

    // Compute center
    trans.transform(center, center);

    Vector3 textends = Vector3.zero();

    trans.r1to0.getRowWith(0, tmp);
    tmp.absolute();
    textends.x = extends_.dot(tmp);

    trans.r1to0.getRowWith(1, tmp);
    tmp.absolute();
    textends.y = extends_.dot(tmp);

    trans.r1to0.getRowWith(2, tmp);
    tmp.absolute();
    textends.z = extends_.dot(tmp);

    min.sub2(center,textends);
    max.add2(center,textends);
  }
  
  /**
   * Merges a Box.
   */
  void merge(AABB box) {
    min.x = Math.min(min.x, box.min.x);
    min.y = Math.min(min.y, box.min.y);
    min.z = Math.min(min.z, box.min.z);

    max.x = Math.max(max.x, box.max.x);
    max.y = Math.max(max.y, box.max.y);
    max.z = Math.max(max.z, box.max.z);
  }

  /**
   * Merges a point.
   */
  void mergePoint(Vector3 point) {
    min.x = Math.min(min.x, point.x);
    min.y = Math.min(min.y, point.y);
    min.z = Math.min(min.z, point.z);

    max.x = Math.max(max.x, point.x);
    max.y = Math.max(max.y, point.y);
    max.z = Math.max(max.z, point.z);
  }
  
  /**
   * Gets the extend and center.
   */
  void getCenterExtend(Vector3 center, Vector3 extend) {
    center.add2(max, min);
    center.scale(0.5);

    extend.sub2(max, center);
  }
  
  /**
   * Finds the intersecting box between this box and the other.
   */
  void findIntersection(AABB other, AABB intersection) {
    intersection.min.x = Math.max(other.min.x, min.x);
    intersection.min.y = Math.max(other.min.y, min.y);
    intersection.min.z = Math.max(other.min.z, min.z);

    intersection.max.x = Math.min(other.max.x, max.x);
    intersection.max.y = Math.min(other.max.y, max.y);
    intersection.max.z = Math.min(other.max.z, max.z);
  }

  bool hasCollision(AABB other) {
    if (min.x > other.max.x ||
        max.x < other.min.x ||
        min.y > other.max.y ||
        max.y < other.min.y ||
        min.z > other.max.z ||
        max.z < other.min.z) {
      return false;
    }
    return true;
  }
  
  /**
   * Finds the Ray intersection parameter.
   *
   * @param vorigin  a vec3f with the origin of the ray
   * @param vdir     a vec3f with the direction of the ray
   */
  bool collideRay(Vector3 vorigin, Vector3 vdir) {
    Vector3 extents = Vector3.zero(), center = Vector3.zero();
    getCenterExtend(center, extents);

    double Dx = vorigin.x - center.x;
    if (greater(Dx, extents.x) && Dx * vdir.x >= 0.0) return false;
    
    double Dy = vorigin.y - center.y;
    if (greater(Dy, extents.y) && Dy * vdir.y >= 0.0) return false;
    
    double Dz = vorigin.z - center.z;
    if (greater(Dz, extents.z) && Dz * vdir.z >= 0) return false;
    
    double f = vdir.y * Dz - vdir.z * Dy;
    if (f.abs() > extents.y * vdir.z.abs() + extents.z * vdir.y.abs()) return false;
    
    f = vdir.z * Dx - vdir.x * Dz;
    if (f.abs() > extents.x * vdir.z.abs() + extents.z * vdir.x.abs()) return false;
    
    f = vdir.x * Dy - vdir.y * Dx;
    if (f.abs() > extents.x * vdir.y.abs() + extents.y * vdir.x.abs()) return false;
    
    return true;
  }

  void projectionInterval(Vector3 direction, List<double> vmin, List<double> vmax) {
    Vector3 tmp = Vector3.zero();

    Vector3 center = Vector3.zero();
    Vector3 extend = Vector3.zero();
    getCenterExtend(center, extend);

    double _fOrigin = direction.dot(center);
    tmp.absoluteFrom(direction);
    double _fMaximumExtent = extend.dot(tmp);
    vmin[0] = _fOrigin - _fMaximumExtent;
    vmax[0] = _fOrigin + _fMaximumExtent;
  }

  PlaneIntersectionType plane_classify(Vector4 plane) {
    Vector3 tmp = Vector3.zero();

    List<double> _fmin = [0], _fmax = [0];
    tmp.setValues(plane.x, plane.y, plane.z);
    projectionInterval(tmp, _fmin, _fmax);

    if (plane.w > _fmax[0] + BoxCollision.boxPlaneEpsilon) {
      return PlaneIntersectionType.backPlane; // 0
    }

    if (plane.w + BoxCollision.boxPlaneEpsilon >= _fmin[0]) {
      return PlaneIntersectionType.collidePlane; //1
    }
    
    return PlaneIntersectionType.frontPlane; //2
  }
  
  bool overlappingTransConservative(AABB box, Transform trans1To0) {
    AABB tbox = AABB.fromAABB(box);
    tbox.appyTransform(trans1To0);
    return hasCollision(tbox);
  }

  bool overlappingTransConservative2(AABB box, BoxBoxTransformCache trans1To0) {
    AABB tbox = AABB.fromAABB(box);
    tbox.appyTransformTransCache(trans1To0);
    return hasCollision(tbox);
  }

  /**
   * transcache is the transformation cache from box to this AABB.
   */
  bool overlappingTransCache(AABB box, BoxBoxTransformCache transcache, bool fulltest) {
    Vector3 tmp = Vector3.zero();

    // Taken from OPCODE
    Vector3 ea = Vector3.zero(), eb = Vector3.zero(); //extends
    Vector3 ca = Vector3.zero(), cb = Vector3.zero(); //extends
    getCenterExtend(ca, ea);
    box.getCenterExtend(cb, eb);

    Vector3 T = Vector3.zero();
    double t, t2;

    // Class I : A's basis vectors
    for (int i=0; i<3; i++) {
      transcache.r1to0.getRowWith(i, tmp);
      VectorUtil.setCoord(T, i, tmp.dot(cb) + VectorUtil.getCoord(transcache.t1to0, i) - VectorUtil.getCoord(ca, i));

      transcache.ar.getRowWith(i, tmp);
      t = tmp.dot(eb) + VectorUtil.getCoord(ea, i);
      if (greater(VectorUtil.getCoord(T, i), t)) {
        return false;
      }
    }
    // Class II : B's basis vectors
    for (int i=0; i<3; i++) {
      t = BoxCollision.btMat3DotCol(transcache.r1to0, T, i);
      t2 = BoxCollision.btMat3DotCol(transcache.ar, ea, i) + VectorUtil.getCoord(eb, i);
      if (greater(t, t2)) {
        return false;
      }
    }
    // Class III : 9 cross products
    if (fulltest) {
      int m, n, o, p, q, r;
      for (int i = 0; i < 3; i++) {
        m = (i+1) % 3;
        n = (i+2) % 3;
        o = (i == 0)? 1:0;
        p = (i == 2)? 1:2;
        for (int j=0; j<3; j++) {
          q = j == 2 ? 1 : 2;
          r = j == 0 ? 1 : 0;
          t = VectorUtil.getCoord(T, n) * transcache.r1to0.getElement(m, j) - VectorUtil.getCoord(T, m) * transcache.r1to0.getElement(n, j);
          t2 = VectorUtil.getCoord(ea, o) * transcache.ar.getElement(p, j) + VectorUtil.getCoord(ea, p) * transcache.ar.getElement(o, j) +
              VectorUtil.getCoord(eb, r) * transcache.ar.getElement(i, q) + VectorUtil.getCoord(eb, q) * transcache.ar.getElement(i, r);
          if (greater(t, t2)) {
            return false;
          }
        }
      }
    }
    return true;
  }
  
  /**
   * Simple test for planes.
   */
  bool collidePlane(Vector4 plane) {
    PlaneIntersectionType classify = plane_classify(plane);
    return (classify == PlaneIntersectionType.collidePlane);
  }
  
  /**
   * Test for a triangle, with edges.
   */
  bool collideTriangleExact(Vector3 p1, Vector3 p2, Vector3 p3, Vector4 trianglePlane) {
    if (!collidePlane(trianglePlane)) {
      return false;
    }
    Vector3 center = Vector3.zero(), extends_ = Vector3.zero();
    getCenterExtend(center, extends_);

    Vector3 v1 = Vector3.zero();
    v1.sub2(p1,center);
    Vector3 v2 = Vector3.zero();
    v2.sub2(p2,center);
    Vector3 v3 = Vector3.zero();
    v3.sub2(p3,center);

    // First axis
    Vector3 diff = Vector3.zero();
    diff.sub2(v2,v1);
    Vector3 absDiff = Vector3.zero();
    absDiff.absoluteFrom(diff);

    // Test With X axis
    BoxCollision.testCrossEdgeBoxXAxisMCR(diff, absDiff, v1, v3, extends_);
    // Test With Y axis
    BoxCollision.testCrossEdgeBoxYAxisMCR(diff, absDiff, v1, v3, extends_);
    // Test With Z axis
    BoxCollision.testCrossEdgeBoxZAxisMCR(diff, absDiff, v1, v3, extends_);

    diff.sub2(v3,v2);
    absDiff.absoluteFrom(diff);

    // Test With X axis
    BoxCollision.testCrossEdgeBoxXAxisMCR(diff, absDiff, v2, v1, extends_);
    // Test With Y axis
    BoxCollision.testCrossEdgeBoxYAxisMCR(diff, absDiff, v2, v1, extends_);
    // Test With Z axis
    BoxCollision.testCrossEdgeBoxZAxisMCR(diff, absDiff, v2, v1, extends_);

    diff.sub2(v1,v3);
    absDiff.absoluteFrom(diff);

    // Test With X axis
    BoxCollision.testCrossEdgeBoxXAxisMCR(diff, absDiff, v3, v2, extends_);
    // Test With Y axis
    BoxCollision.testCrossEdgeBoxYAxisMCR(diff, absDiff, v3, v2, extends_);
    // Test With Z axis
    BoxCollision.testCrossEdgeBoxZAxisMCR(diff, absDiff, v3, v2, extends_);

    return true;
  }
}