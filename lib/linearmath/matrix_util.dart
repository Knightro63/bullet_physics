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
import '../core/bullet_globals.dart';
import 'package:vector_math/vector_math.dart';
import 'dart:math';

extension Mat3 on Matrix3{
  void transformFrom(Vector3 v1, Vector3 v2) {
    final argStorage = v1.storage;
    final x_ = 
        (storage[0] * argStorage[0]) +
        (storage[3] * argStorage[1]) +
        (storage[6] * argStorage[2]);
    final y_ = 
        (storage[1] * argStorage[0]) +
        (storage[4] * argStorage[1]) +
        (storage[7] * argStorage[2]);
    final z_ = 
        (storage[2] * argStorage[0]) +
        (storage[5] * argStorage[1]) +
        (storage[8] * argStorage[2]);
    v2
      ..x = x_
      ..y = y_
      ..z = z_;
  }
  void mul(Matrix3 m){
    final m00 = storage[0];
    final m01 = storage[3];
    final m02 = storage[6];
    final m10 = storage[1];
    final m11 = storage[4];
    final m12 = storage[7];
    final m20 = storage[2];
    final m21 = storage[5];
    final m22 = storage[8];
    final argStorage = m.storage;
    final n00 = argStorage[0];
    final n01 = argStorage[3];
    final n02 = argStorage[6];
    final n10 = argStorage[1];
    final n11 = argStorage[4];
    final n12 = argStorage[7];
    final n20 = argStorage[2];
    final n21 = argStorage[5];
    final n22 = argStorage[8];

    storage[0] = (m00 * n00) + (m01 * n10) + (m02 * n20);
    storage[3] = (m00 * n01) + (m01 * n11) + (m02 * n21);
    storage[6] = (m00 * n02) + (m01 * n12) + (m02 * n22);
    storage[1] = (m10 * n00) + (m11 * n10) + (m12 * n20);
    storage[4] = (m10 * n01) + (m11 * n11) + (m12 * n21);
    storage[7] = (m10 * n02) + (m11 * n12) + (m12 * n22);
    storage[2] = (m20 * n00) + (m21 * n10) + (m22 * n20);
    storage[5] = (m20 * n01) + (m21 * n11) + (m22 * n21);
    storage[8] = (m20 * n02) + (m21 * n12) + (m22 * n22);
  }
  void mul2(Matrix3 m1, Matrix3 m2){
    final m00 = m1.storage[0];
    final m01 = m1.storage[3];
    final m02 = m1.storage[6];
    final m10 = m1.storage[1];
    final m11 = m1.storage[4];
    final m12 = m1.storage[7];
    final m20 = m1.storage[2];
    final m21 = m1.storage[5];
    final m22 = m1.storage[8];
    final argStorage = m2.storage;
    final n00 = argStorage[0];
    final n01 = argStorage[3];
    final n02 = argStorage[6];
    final n10 = argStorage[1];
    final n11 = argStorage[4];
    final n12 = argStorage[7];
    final n20 = argStorage[2];
    final n21 = argStorage[5];
    final n22 = argStorage[8];

    storage[0] = (m00 * n00) + (m01 * n10) + (m02 * n20);
    storage[3] = (m00 * n01) + (m01 * n11) + (m02 * n21);
    storage[6] = (m00 * n02) + (m01 * n12) + (m02 * n22);
    storage[1] = (m10 * n00) + (m11 * n10) + (m12 * n20);
    storage[4] = (m10 * n01) + (m11 * n11) + (m12 * n21);
    storage[7] = (m10 * n02) + (m11 * n12) + (m12 * n22);
    storage[2] = (m20 * n00) + (m21 * n10) + (m22 * n20);
    storage[5] = (m20 * n01) + (m21 * n11) + (m22 * n21);
    storage[8] = (m20 * n02) + (m21 * n12) + (m22 * n22);
  }
  /// Transpose this.
  void transposeFrom(Matrix3 m) {
    double temp;
    temp = m.storage[3];
    storage[3] = m.storage[1];
    storage[1] = temp;
    temp = m.storage[6];
    storage[6] = m.storage[2];
    storage[2] = temp;
    temp = m.storage[7];
    storage[7] = m.storage[5];
    storage[5] = temp;
  }
  void getRowWith(int row, final Vector3 v){
    v.setFrom(getRow(row));
  }
  void getColumnWith(int col, final Vector3 v){
    v.setFrom(getColumn(col));
  }
  double getElement(int row, int col){
    return storage[row*3+col];
  }
  void setElement(int row, int col, double val){
    storage[row*3+col] = val;
  }
  void setRowByValues(int row, double val1, double val2, double val3){
    storage[row*3+0] = val1;
    storage[row*3+1] = val2;
    storage[row*3+2] = val3;
  }
  bool equals(Matrix3 m1){
    return m1.storage[0] == storage[0] && 
      m1.storage[1] == storage[1] &&
      m1.storage[2] == storage[2] && 
      m1.storage[3] == storage[3] && 
      m1.storage[4] == storage[4] && 
      m1.storage[5] == storage[5] && 
      m1.storage[6] == storage[6] && 
      m1.storage[7] == storage[7] && 
      m1.storage[8] == storage[8];
  }
}

extension Mat4 on Matrix4{
  Matrix3 getUpper3x3(){
    return Matrix3(
      storage[0],
      storage[1],
      storage[2],
      storage[4],
      storage[5],
      storage[6],
      storage[8],
      storage[9],
      storage[10]
    );
  }

  /// Sets the upper 2x2 of the matrix to be [arg].
  void setFromMatrix3(Matrix3 arg) {
    final argStorage = arg.storage;
    storage[0] = argStorage[0];
    storage[1] = argStorage[1];
    storage[2] = argStorage[2];
    storage[3] = 0;

    storage[4] = argStorage[3];
    storage[5] = argStorage[4];
    storage[6] = argStorage[5];
    storage[7] = 0;

    storage[8] = argStorage[6];
    storage[9] = argStorage[7];
    storage[10] = argStorage[8];
    storage[11] = 0;

    storage[12] = 0;
    storage[13] = 0;
    storage[14] = 0;
    storage[15] = 1;
  }
}

/**
 * Utility functions for matrices.
 * 
 * @author jezek2
 */
class MatrixUtil {
	
	static void scale(Matrix3 dest, Matrix3 mat, Vector3 s) {
		dest.storage[0] = mat.storage[0] * s.x;   dest.storage[1] = mat.storage[1] * s.y;   dest.storage[2] = mat.storage[2] * s.z;
		dest.storage[3] = mat.storage[3] * s.x;   dest.storage[4] = mat.storage[4] * s.y;   dest.storage[5] = mat.storage[5] * s.z;
		dest.storage[6] = mat.storage[6] * s.x;   dest.storage[7] = mat.storage[7] * s.y;   dest.storage[8] = mat.storage[8] * s.z;
	}
	
	static void absolute(Matrix3 mat) {
		mat.storage[0] = mat.storage[0].abs();
		mat.storage[1] = mat.storage[1].abs();
		mat.storage[2] = mat.storage[2].abs();
		mat.storage[3] = mat.storage[3].abs();
		mat.storage[4] = mat.storage[4].abs();
		mat.storage[5] = mat.storage[5].abs();
		mat.storage[6] = mat.storage[6].abs();
		mat.storage[7] = mat.storage[7].abs();
		mat.storage[8] = mat.storage[8].abs();
	}
	
	static void setFromOpenGLSubMatrix(Matrix3 mat, List<double> m) {
		mat.storage[0] = m[0]; mat.storage[1] = m[4]; mat.storage[2] = m[8];
		mat.storage[3] = m[1]; mat.storage[4] = m[5]; mat.storage[5] = m[9];
		mat.storage[6] = m[2]; mat.storage[7] = m[6]; mat.storage[8] = m[10];
	}

	static void getOpenGLSubMatrix(Matrix3 mat, List<double> m) {
		m[0] = mat.storage[0];
		m[1] = mat.storage[3];
		m[2] = mat.storage[6];
		m[3] = 0;
		m[4] = mat.storage[1];
		m[5] = mat.storage[4];
		m[6] = mat.storage[7];
		m[7] = 0;
		m[8] = mat.storage[2];
		m[9] = mat.storage[5];
		m[10] = mat.storage[8];
		m[11] = 0;
	}
	
	/**
	 * Sets rotation matrix from euler angles. The euler angles are applied in ZYX
	 * order. This means a vector is first rotated about X then Y and then Z axis.
	 */
	static void setEulerZYX(Matrix3 mat, double eulerX, double eulerY, double eulerZ) {
		double ci = cos(eulerX);
		double cj = cos(eulerY);
		double ch = cos(eulerZ);
		double si = sin(eulerX);
		double sj = sin(eulerY);
		double sh = sin(eulerZ);
		double cc = ci * ch;
		double cs = ci * sh;
		double sc = si * ch;
		double ss = si * sh;

		mat.setRowByValues(0, cj * ch, sj * sc - cs, sj * cc + ss);
		mat.setRowByValues(1, cj * sh, sj * ss + cc, sj * cs - sc);
		mat.setRowByValues(2, -sj, cj * si, cj * ci);
	}
	
	static double _tdotx(Matrix3 mat, Vector3 vec) {
		return mat.storage[0] * vec.x + mat.storage[3] * vec.y + mat.storage[6] * vec.z;
	}

	static double _tdoty(Matrix3 mat, Vector3 vec) {
		return mat.storage[1] * vec.x + mat.storage[4] * vec.y + mat.storage[7] * vec.z;
	}

	static double _tdotz(Matrix3 mat, Vector3 vec) {
		return mat.storage[2] * vec.x + mat.storage[5] * vec.y + mat.storage[8] * vec.z;
	}
	
	static void transposeTransform(Vector3 dest, Vector3 vec, Matrix3 mat) {
		double x = _tdotx(mat, vec);
		double y = _tdoty(mat, vec);
		double z = _tdotz(mat, vec);
		dest.x = x;
		dest.y = y;
		dest.z = z;
	}
	
	static void setRotation(Matrix3 dest, Quaternion q) {
		double d = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		assert (d != 0);
		double s = 2 / d;
		double xs = q.x * s, ys = q.y * s, zs = q.z * s;
		double wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
		double xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
		double yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
		dest.storage[0] = 1 - (yy + zz);
		dest.storage[1] = xy - wz;
		dest.storage[2] = xz + wy;
		dest.storage[3] = xy + wz;
		dest.storage[4] = 1 - (xx + zz);
		dest.storage[5] = yz - wx;
		dest.storage[6] = xz - wy;
		dest.storage[7] = yz + wx;
		dest.storage[8] = 1 - (xx + yy);
	}
	
	static void getRotation(Matrix3 mat, Quaternion dest) {
		//List<double> doubleArrays = ArrayPool.get(double.class);
		
		double trace = mat.storage[0] + mat.storage[4] + mat.storage[8];
		List<double> temp = List.filled(4, 0);//doubleArrays.getFixed(4);

		if (trace > 0) {
			double s =sqrt(trace + 1);
			temp[3] = (s * 0.5);
			s = 0.5 / s;

			temp[0] = ((mat.storage[7] - mat.storage[5]) * s);
			temp[1] = ((mat.storage[2] - mat.storage[6]) * s);
			temp[2] = ((mat.storage[3] - mat.storage[1]) * s);
		}
		else {
			int i = mat.storage[0] < mat.storage[4] ? (mat.storage[4] < mat.storage[8] ? 2 : 1) : (mat.storage[0] < mat.storage[8] ? 2 : 0);
			int j = (i + 1) % 3;
			int k = (i + 2) % 3;

			double s = sqrt(mat.getElement(i, i) - mat.getElement(j, j) - mat.getElement(k, k) + 1);
			temp[i] = s * 0.5;
			s = 0.5 / s;

			temp[3] = (mat.getElement(k, j) - mat.getElement(j, k)) * s;
			temp[j] = (mat.getElement(j, i) + mat.getElement(i, j)) * s;
			temp[k] = (mat.getElement(k, i) + mat.getElement(i, k)) * s;
		}
		dest.setValues(temp[0], temp[1], temp[2], temp[3]);
		
		//doubleArrays.release(temp);
	}

	static double _cofac(Matrix3 mat, int r1, int c1, int r2, int c2) {
		return mat.getElement(r1, c1) * mat.getElement(r2, c2) - mat.getElement(r1, c2) * mat.getElement(r2, c1);
	}
	
	static void invert(Matrix3 mat) {
		double co_x = _cofac(mat, 1, 1, 2, 2);
		double co_y = _cofac(mat, 1, 2, 2, 0);
		double co_z = _cofac(mat, 1, 0, 2, 1);
		
		double det = mat.storage[0]*co_x + mat.storage[1]*co_y + mat.storage[2]*co_z;
		assert (det != 0);
		
		double s = 1 / det;
		List<double> storage = [co_x * s,_cofac(mat, 0, 2, 2, 1) * s,_cofac(mat, 0, 1, 1, 2) * s,co_y * s,_cofac(mat, 0, 0, 2, 2) * s,_cofac(mat, 0, 2, 1, 0) * s,co_z * s,_cofac(mat, 0, 1, 2, 0) * s,_cofac(mat, 0, 0, 1, 1) * s];
		
		mat.storage[0] = storage[0];
		mat.storage[1] = storage[1];
		mat.storage[2] = storage[2];
		mat.storage[3] = storage[3];
		mat.storage[4] = storage[4];
		mat.storage[5] = storage[5];
		mat.storage[6] = storage[6];
		mat.storage[7] = storage[7];
		mat.storage[8] = storage[8];
	}

	/**
	 * Diagonalizes this matrix by the Jacobi method. rot stores the rotation
	 * from the coordinate system in which the matrix is diagonal to the original
	 * coordinate system, i.e., old_this = rot * new_this * rot^T. The iteration
	 * stops when all off-diagonal elements are less than the threshold multiplied
	 * by the sum of the absolute values of the diagonal, or when maxSteps have
	 * been executed. Note that this matrix is assumed to be symmetric.
	 */
	// JAVA NOTE: diagonalize method from 2.71
	static void diagonalize(Matrix3 mat, Matrix3 rot, double threshold, int maxSteps) {
		final Vector3 row = Vector3.zero();

		rot.setIdentity();
		for (int step = maxSteps; step > 0; step--) {
			// find off-diagonal element [p][q] with largest magnitude
			int p = 0;
			int q = 1;
			int r = 2;
			double max = mat.storage[1].abs();
			double v =mat.storage[2].abs();
			if (v > max) {
				q = 2;
				r = 1;
				max = v;
			}
			v = mat.storage[5].abs();
			if (v > max) {
				p = 1;
				q = 2;
				r = 0;
				max = v;
			}

			double t = threshold * (mat.storage[0].abs() +mat.storage[4].abs() +mat.storage[8].abs());
			if (max <= t) {
				if (max <= BulletGlobals.simdEpsilon * t) {
					return;
				}
				step = 1;
			}

			// compute Jacobi rotation J which leads to a zero for element [p][q]
			double mpq = mat.getElement(p, q);
			double theta = (mat.getElement(q, q) - mat.getElement(p, p)) / (2 * mpq);
			double theta2 = theta * theta;
			double cos;
			double sin;
			if ((theta2 * theta2) < (10/ BulletGlobals.simdEpsilon)) {
				t = (theta >= 0) ? 1 / (theta + sqrt(1 + theta2))
						: 1 / (theta - sqrt(1 + theta2));
				cos = 1 / sqrt(1 + t * t);
				sin = cos * t;
			}
			else {
				// approximation for large theta-value, i.e., a nearly diagonal matrix
				t = 1 / (theta * (2 + 0.5 / theta2));
				cos = 1 - 0.5 * t * t;
				sin = cos * t;
			}

			// apply rotation to matrix (this = J^T * this * J)
			mat.setElement(p, q, 0);
			mat.setElement(q, p, 0);
			mat.setElement(p, p, mat.getElement(p, p) - t * mpq);
			mat.setElement(q, q, mat.getElement(q, q) + t * mpq);
			double mrp = mat.getElement(r, p);
			double mrq = mat.getElement(r, q);
			mat.setElement(r, p, cos * mrp - sin * mrq);
			mat.setElement(p, r, cos * mrp - sin * mrq);
			mat.setElement(r, q, cos * mrq + sin * mrp);
			mat.setElement(q, r, cos * mrq + sin * mrp);

			// apply rotation to rot (rot = rot * J)
			for (int i=0; i<3; i++) {
				row.setFrom(rot.getRow(i));

				mrp = VectorUtil.getCoord(row, p);
				mrq = VectorUtil.getCoord(row, q);
				VectorUtil.setCoord(row, p, cos * mrp - sin * mrq);
				VectorUtil.setCoord(row, q, cos * mrq + sin * mrp);
				rot.setRow(i, row);
			}
		}
	}
}
