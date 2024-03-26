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

import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class BoxCollision {
	static const double boxPlaneEpsilon = 0.000001;
	
	static bool testCrossEdgeBoxMCR(Vector3 edge, Vector3 absoluteEdge, Vector3 pointa, Vector3 pointb, Vector3 extend, int i_dir_0, int i_dir_1, int i_comp_0, int i_comp_1) {
		double dir0 = -VectorUtil.getCoord(edge, i_dir_0);
		double dir1 = VectorUtil.getCoord(edge, i_dir_1);
		double pmin = VectorUtil.getCoord(pointa, i_comp_0) * dir0 + VectorUtil.getCoord(pointa, i_comp_1) * dir1;
		double pmax = VectorUtil.getCoord(pointb, i_comp_0) * dir0 + VectorUtil.getCoord(pointb, i_comp_1) * dir1;
		if (pmin > pmax) {
			pmin = pmin + pmax;
			pmax = pmin - pmax;
			pmin = pmin - pmax;
		}
		double absDir0 = VectorUtil.getCoord(absoluteEdge, i_dir_0);
		double absDir1 = VectorUtil.getCoord(absoluteEdge, i_dir_1);
		double rad = VectorUtil.getCoord(extend, i_comp_0) * absDir0 + VectorUtil.getCoord(extend, i_comp_1) * absDir1;
		if (pmin > rad || -rad > pmax) {
			return false;
		}
		return true;
	}

	static bool testCrossEdgeBoxXAxisMCR(Vector3 edge, Vector3 absoluteEdge, Vector3 pointa, Vector3 pointb, Vector3 extend) {
		return testCrossEdgeBoxMCR(edge, absoluteEdge, pointa, pointb, extend, 2, 1, 1, 2);
	}

	static bool testCrossEdgeBoxYAxisMCR(Vector3 edge, Vector3 absoluteEdge, Vector3 pointa, Vector3 pointb, Vector3 extend) {
		return testCrossEdgeBoxMCR(edge, absoluteEdge, pointa, pointb, extend, 0, 2, 2, 0);
	}

	static bool testCrossEdgeBoxZAxisMCR(Vector3 edge, Vector3 absoluteEdge, Vector3 pointa, Vector3 pointb, Vector3 extend) {
		return testCrossEdgeBoxMCR(edge, absoluteEdge, pointa, pointb, extend, 1, 0, 0, 1);
	}
	
	/**
	 * Returns the dot product between a vec3f and the col of a matrix.
	 */
	static double btMat3DotCol(Matrix3 mat, Vector3 vec3, int colindex) {
		return vec3.x*mat.getElement(0, colindex) + vec3.y*mat.getElement(1, colindex) + vec3.z*mat.getElement(2, colindex);
	}

	/**
	 * Compairison of transformation objects.
	 */
	static bool compareTransformsEqual(Transform t1, Transform t2) {
		return t1.equals(t2);
	}
}
	
////////////////////////////////////////////////////////////////////////////

class BoxBoxTransformCache {
  final Vector3 t1to0 = Vector3.zero(); // Transforms translation of model1 to model 0
  final Matrix3 r1to0 = Matrix3.zero(); // Transforms Rotation of model1 to model 0, equal  to R0' * R1
  final Matrix3 ar = Matrix3.zero();    // Absolute value of m_R1to0
  
  void set(BoxBoxTransformCache cache) {
    throw'UnsupportedOperationException()';
  }
  
  void calcAbsoluteMatrix() {
    for (int i=0; i<3; i++) {
      for (int j=0; j<3; j++) {
        ar.setElement(i, j, 1e-6 + (r1to0.getElement(i, j)).abs());
      }
    }
  }

  /**
   * Calc the transformation relative  1 to 0. Inverts matrics by transposing.
   */
  void calcFromHomogenic(Transform? trans0, Transform? trans1) {
    Transform tempTrans = Transform();
    tempTrans.inverse(trans0);
    tempTrans.mul(trans1);

    t1to0.setFrom(tempTrans.origin);
    r1to0.setFrom(tempTrans.basis);

    calcAbsoluteMatrix();
  }
  
  /**
   * Calcs the full invertion of the matrices. Useful for scaling matrices.
   */
  void calcFromFullInvert(Transform trans0, Transform trans1) {
    r1to0.copyInverse(trans0.basis);
    t1to0.negateFrom(trans0.origin);
    r1to0.transform(t1to0);

    Vector3 tmp = Vector3.zero();
    tmp.setFrom(trans1.origin);
    r1to0.transform(tmp);
    t1to0.add(tmp);

    r1to0.mul(trans1.basis);

    calcAbsoluteMatrix();
  }
  
  Vector3 transform(Vector3 point, Vector3 out) {
    if (point == out) {
      point = Vector3.copy(point);
    }
    
    Vector3 tmp = Vector3.zero();
    r1to0.getRowWith(0, tmp);
    out.x = tmp.dot(point) + t1to0.x;
    r1to0.getRowWith(1, tmp);
    out.y = tmp.dot(point) + t1to0.y;
    r1to0.getRowWith(2, tmp);
    out.z = tmp.dot(point) + t1to0.z;
    return out;
  }
}
