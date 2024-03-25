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

import 'package:bullet_physics/linearmath/matrix_util.dart';
import 'package:bullet_physics/linearmath/vector_util.dart';
import 'package:vector_math/vector_math.dart';

/**
 * Transform represents translation and rotation (rigid transform). Scaling and
 * shearing is not supported.<p>
 * 
 * You can use local shape scaling or {@link UniformScalingShape} for static rescaling
 * of collision objects.
 * 
 * @author jezek2
 */
class Transform {
	
	//BulletStack stack;

	/** Rotation matrix of this Transform. */
	final Matrix3 basis = Matrix3.zero();
	/** Translation vector of this Transform. */
	final Vector3 origin = Vector3.zero();

  Transform();

	Transform.formMatrix3(Matrix3 mat) {
		basis.setFrom(mat);
	}

	Transform.formMatrix4(Matrix4 mat) {
		formMatrix4(mat);
	}

	Transform.formTransfrom(Transform? tr) {
    if(tr != null){
		  copy(tr);
    }
	}
	
	void copy(Transform tr) {
		basis.setFrom(tr.basis);
		origin.setFrom(tr.origin);
	}
	
	void setFromMatrix3(Matrix3 mat) {
		basis.setFrom(mat);
		origin.setValues(0, 0, 0);
	}

	void formMatrix4(Matrix4 mat) {
		basis.setFrom(mat.getUpper3x3());
		origin.setValues(mat.storage[3], mat.storage[7], mat.storage[11]);
	}
	
	void transform(Vector3 v) {
		basis.transform(v);
		v.add(origin);
	}

	void setIdentity() {
		basis.setIdentity();
		origin.setValues(0, 0, 0);
	}

	void inverse([Transform? tr]) {
    if(tr != null){
		  copy(tr);
    }
		basis.transpose();
		origin.scale(-1);
		basis.transform(origin);
	}

	void mul(Transform? tr, [Transform? tr2]) {
    if(tr == null) return;
    Vector3 vec = tr2 == null?Vector3.copy(tr.origin):Vector3.copy(tr2.origin);
    tr.transform(vec);
    if(tr2?.basis != null){
      basis.multiply(tr.basis*tr2?.basis);
    }
    origin.setFrom(vec);
	}
	
	void invXform(Vector3 inVec, Vector3 out) {
		out.sub2(inVec,origin);

		Matrix3 mat = Matrix3.copy(basis);
		mat.transpose();
		mat.transform(out);
	}
	
	Quaternion getRotation(Quaternion out) {
		MatrixUtil.getRotation(basis, out);
		return out;
	}
	
	void setRotation(Quaternion q) {
		MatrixUtil.setRotation(basis, q);
	}
	
	void setFromOpenGLMatrix(List<double> m) {
		MatrixUtil.setFromOpenGLSubMatrix(basis, m);
		origin.setValues(m[12], m[13], m[14]);
	}

	void getOpenGLMatrix(List<double> m) {
		MatrixUtil.getOpenGLSubMatrix(basis, m);
		m[12] = origin.x;
		m[13] = origin.y;
		m[14] = origin.z;
		m[15] = 1;
	}

	Matrix4 getMatrix(Matrix4 out) {
		out.setFromMatrix3(basis);
		out.setEntry(0, 1, origin.x);
		out.setEntry(1, 3, origin.y);
		out.setEntry(2, 3, origin.z);
		return out;
	}

	bool equals(Object? obj) {
		if (obj == null || (obj is! Transform)) return false;
		Transform tr = obj;
		return basis.equals(tr.basis) && origin.equals(tr.origin);
	}

	@override
	int get hashCode => _getHash();
  int _getHash(){
		int hash = 3;
		hash = 41 * hash + basis.hashCode;
		hash = 41 * hash + origin.hashCode;
		return hash;
	}

  /// Check if two matrices are the same.
  @override
  bool operator ==(Object other) =>
      (other is Transform) &&
      (basis.storage[0] == other.basis.storage[0]) &&
      (basis.storage[1] == other.basis.storage[1]) &&
      (basis.storage[2] == other.basis.storage[2]) &&
      (basis.storage[3] == other.basis.storage[3]) &&
      (basis.storage[4] == other.basis.storage[4]) &&
      (basis.storage[5] == other.basis.storage[5]) &&
      (basis.storage[6] == other.basis.storage[6]) &&
      (basis.storage[7] == other.basis.storage[7]) &&
      (basis.storage[8] == other.basis.storage[8]) &&
      origin.storage[0] == other.origin.storage[0] &&
      origin.storage[1] == other.origin.storage[1] &&
      origin.storage[2] == other.origin.storage[2];
}
