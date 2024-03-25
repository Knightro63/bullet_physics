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

import 'package:vector_math/vector_math.dart';
import 'dart:math';

extension Vec4 on Vector4{
  void normalizeFrom(Vector4 v){
    final l = v.length;
    if (l == 0.0) {
      return;
    }
    final d = 1.0 / l;
    storage[0] *= d;
    storage[1] *= d;
    storage[2] *= d;
    storage[3] *= d;
  }

  /// Set the values by copying them from [other].
  void setFromVector3(Vector3 other) {
    final otherStorage = other.storage;
    //storage[3] = otherStorage[3];
    storage[2] = otherStorage[2];
    storage[1] = otherStorage[1];
    storage[0] = otherStorage[0];
  }
}

extension Vec3 on Vector3{
  void interpolate(Vector3 v1, Vector3 v2, double alpha){
    storage[0] = (1-alpha)*v1.storage[0]+alpha*v2.storage[0];
    storage[1] = (1-alpha)*v1.storage[1]+alpha*v2.storage[1];
    storage[2] = (1-alpha)*v1.storage[2]+alpha*v2.storage[2];
  }
  /// Negate this.
  void negateFrom(Vector3 v){
    storage[2] = -v.storage[2];
    storage[1] = -v.storage[1];
    storage[0] = -v.storage[0];
  }
  void normalizeFrom(Vector3 v){
    final l = v.length;
    if (l == 0.0) {
      return;
    }
    final d = 1.0 / l;
    storage[0] *= d;
    storage[1] *= d;
    storage[2] *= d;
  }
  bool equals(Vector3 v){
    return x == v.x && y == v.y && z==v.z;
  }
  /// Cross product.
  void cross2(Vector3? v1, Vector3? v2) {
    if(v1 == null || v2 == null) return;
    final ix = v1.storage[0];
    final iy = v1.storage[1];
    final iz = v1.storage[2];
    final otherStorage = v2.storage;
    final ox = otherStorage[0];
    final oy = otherStorage[1];
    final oz = otherStorage[2];
    
    storage[0] = iy * oz - iz * oy; 
    storage[1] = iz * ox - ix * oz; 
    storage[2] = ix * oy - iy * ox;
  }

  void scaleAdd(double s, Vector3 v1, Vector3 v2){
    storage[0] = v1.storage[0]*s+v2.storage[0];
    storage[1] = v1.storage[1]*s+v2.storage[1];
    storage[2] = v1.storage[2]*s+v2.storage[2];
  }

  void scaleFrom(double s, Vector3? v){
    if(v == null) return;
    storage[0] = v.storage[0]*s;
    storage[1] = v.storage[1]*s;
    storage[2] = v.storage[2]*s;
  }

  void add2(Vector3? v1, Vector3? v2){
    if(v1 == null || v2 == null) return;
    final ix = v1.storage[0];
    final iy = v1.storage[1];
    final iz = v1.storage[2];
    final otherStorage = v2.storage;
    final ox = otherStorage[0];
    final oy = otherStorage[1];
    final oz = otherStorage[2];
    
    storage[0] = ix + ox; 
    storage[1] = iy + oy; 
    storage[2] = iz + oz;
  }

  void sub2(Vector3? v1, Vector3? v2){
    if(v1 == null || v2 == null) return;
    final ix = v1.storage[0];
    final iy = v1.storage[1];
    final iz = v1.storage[2];
    final otherStorage = v2.storage;
    final ox = otherStorage[0];
    final oy = otherStorage[1];
    final oz = otherStorage[2];
    
    storage[0] = ix - ox; 
    storage[1] = iy - oy; 
    storage[2] = iz - oz;
  }

  void absoluteFrom(Vector3 v){
    storage[0] = v.storage[0].abs(); 
    storage[1] = v.storage[1].abs();
    storage[2] = v.storage[2].abs();
  }
}

/**
 * Utility functions for vectors.
 * 
 * @author jezek2
 */
class VectorUtil {
	static int maxAxis(Vector3 v) {
		int maxIndex = -1;
		double maxVal = -1e30;
		if (v.x > maxVal) {
			maxIndex = 0;
			maxVal = v.x;
		}
		if (v.y > maxVal) {
			maxIndex = 1;
			maxVal = v.y;
		}
		if (v.z > maxVal) {
			maxIndex = 2;
			maxVal = v.z;
		}

		return maxIndex;
	}
	
	static int maxAxis4(Vector4 v) {
		int maxIndex = -1;
		double maxVal = -1e30;
		if (v.x > maxVal) {
			maxIndex = 0;
			maxVal = v.x;
		}
		if (v.y > maxVal) {
			maxIndex = 1;
			maxVal = v.y;
		}
		if (v.z > maxVal) {
			maxIndex = 2;
			maxVal = v.z;
		}
		if (v.w > maxVal) {
			maxIndex = 3;
			maxVal = v.w;
		}

		return maxIndex;
	}

	static int closestAxis4(Vector4 vec) {
		Vector4 tmp = Vector4.copy(vec);
		tmp.absolute();
		return maxAxis4(tmp);
	}
	
	static double getCoord(Vector3 vec, int num) {
		switch (num) {
			case 0: return vec.x;
			case 1: return vec.y;
			case 2: return vec.z;
			default: throw 'num is 0,1, or 2';
		}
	}
	
	static void setCoord(Vector3 vec, int num, double value) {
		switch (num) {
			case 0: vec.x = value; break;
			case 1: vec.y = value; break;
			case 2: vec.z = value; break;
			default: throw 'num is 0,1, or 2';
		}
	}

	static void mulCoord(Vector3 vec, int num, double value) {
		switch (num) {
			case 0: vec.x *= value; break;
			case 1: vec.y *= value; break;
			case 2: vec.z *= value; break;
			default: throw 'num is 0,1, or 2';
		}
	}

	static void setInterpolate3(Vector3 dest, Vector3 v0, Vector3 v1, double rt) {
		double s = 1 - rt;
		dest.x = s * v0.x + rt * v1.x;
		dest.y = s * v0.y + rt * v1.y;
		dest.z = s * v0.z + rt * v1.z;
		// don't do the unused w component
		//		m_co[3] = s * v0[3] + rt * v1[3];
	}

	static void sub(Vector3 dest, Vector3 v1, Vector3 v2) {
		dest.x = v1.x - v2.x;
		dest.y = v1.y - v2.y;
		dest.z = v1.z - v2.z;
	}
	
	static void add(Vector3 dest, Vector3 v1, [Vector3? v2, Vector3? v3, Vector3? v4]){
    v2 ??= Vector3(0,0,0);
    v3 ??= Vector3(0,0,0);
    v4 ??= Vector3(0,0,0);
		dest.x = v1.x + v2.x + v3.x + v4.x;
		dest.y = v1.y + v2.y + v3.y + v4.y;
		dest.z = v1.z + v2.z + v3.z + v4.z;
	}
	
	static void mul(Vector3 dest, Vector3 v1, Vector3 v2) {
		dest.x = v1.x * v2.x;
		dest.y = v1.y * v2.y;
		dest.z = v1.z * v2.z;
	}

	static void scale(Vector3 dest, Vector3 v1, double s) {
		dest.x = v1.x * s;
		dest.y = v1.y * s;
		dest.z = v1.z * s;
	}

	
	static void div(Vector3 dest, Vector3 v1, Vector3 v2) {
		dest.x = v1.x / v2.x;
		dest.y = v1.y / v2.y;
		dest.z = v1.z / v2.z;
	}
	
	static void setMin(Vector3 a, Vector3 b) {
		a.x = min(a.x, b.x);
		a.y = min(a.y, b.y);
		a.z = min(a.z, b.z);
	}
	
	static void setMax(Vector3 a, Vector3 b) {
		a.x = max(a.x, b.x);
		a.y = max(a.y, b.y);
		a.z = max(a.z, b.z);
	}
	
	// static double dot3(Vector4 v0, Vector3 v1) {
	// 	return (v0.x*v1.x + v0.y*v1.y + v0.z*v1.z);
	// }

	// static double dot3(Vector4 v0, Vector4 v1) {
	// 	return (v0.x*v1.x + v0.y*v1.y + v0.z*v1.z);
	// }

	static double dot3(Vector? v0, Vector v1) {
    if(v0 is Vector3 && v1 is Vector4){
		  return (v0.x*v1.x + v0.y*v1.y + v0.z*v1.z);
    }
    else if(v0 is Vector4 && v1 is Vector3){
		  return (v0.x*v1.x + v0.y*v1.y + v0.z*v1.z);
    }
    else if(v0 is Vector4 && v1 is Vector4){
		  return (v0.x*v1.x + v0.y*v1.y + v0.z*v1.z);
    }
    else if(v0 is Vector3 && v1 is Vector3){
		  return (v0.x*v1.x + v0.y*v1.y + v0.z*v1.z);
    }
    throw 'v0 and v1 must be either Vector3 or Vector4';
	}

	static double lengthSquared3(Vector4 v) {
		return (v.x*v.x + v.y*v.y + v.z*v.z);
	}

	static void normalize3(Vector4 v) {
		double norm = (1.0/sqrt(v.x*v.x + v.y*v.y + v.z*v.z));
		v.x *= norm;
		v.y *= norm;
		v.z *= norm;
	}

	static void cross3(Vector3 dest, Vector4 v1, Vector4 v2) {
        double x,y;
        x = v1.y*v2.z - v1.z*v2.y;
        y = v2.x*v1.z - v2.z*v1.x;
        dest.z = v1.x*v2.y - v1.y*v2.x;
        dest.x = x;
        dest.y = y;
	}
}
