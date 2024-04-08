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

import '../core/bullet_globals.dart';
import './vector_util.dart';
import 'package:vector_math/vector_math.dart';
import 'dart:math';

extension Quat4 on Quaternion{
  void multiply(Quaternion q){
    final iw = storage[3];
    final iz = storage[2];
    final iy = storage[1];
    final ix = storage[0];

    final ow = q.storage[3];
    final oz = q.storage[2];
    final oy = q.storage[1];
    final ox = q.storage[0];

    storage[0] = iw * ox + ix * ow + iy * oz - iz * oy;
    storage[1] = iw * oy + iy * ow + iz * ox - ix * oz;
    storage[2] = iw * oz + iz * ow + ix * oy - iy * ox;
    storage[3] = iw * ow - ix * ox - iy * oy - iz * oz;
  }

  void multiply2(Quaternion q1, Quaternion q2){
    final iw = q1.storage[3];
    final iz = q1.storage[2];
    final iy = q1.storage[1];
    final ix = q1.storage[0];

    final ow = q2.storage[3];
    final oz = q2.storage[2];
    final oy = q2.storage[1];
    final ox = q2.storage[0];

    storage[0] = iw * ox + ix * ow + iy * oz - iz * oy;
    storage[1] = iw * oy + iy * ow + iz * ox - ix * oz;
    storage[2] = iw * oz + iz * ow + ix * oy - iy * ox;
    storage[3] = iw * ow - ix * ox - iy * oy - iz * oz;
  }
}

class QuaternionUtil {
	static double getAngle(Quaternion q) {
		double s = 2 * acos(q.w);
		return s;
	}
	
	static void setRotation(Quaternion q, Vector3 axis, double angle) {
		double d = axis.length;
		assert (d != 0);
		double s = sin(angle * 0.5) / d;
		q.setValues(axis.x * s, axis.y * s, axis.z * s, cos(angle * 0.5));
	}
	
	// Game Programming Gems 2.10. make sure v0,v1 are normalized
	static Quaternion intestArcQuat(Vector3 v0, Vector3 v1, Quaternion out) {
		Vector3 c = Vector3.zero();
		c.cross2(v0, v1);
		double d = v0.dot(v1);

		if (d < -1.0 + BulletGlobals.fltEpsilon) {
			// just pick any vector
			out.setValues(0.0, 1.0, 0.0, 0.0);
			return out;
		}

		double s = sqrt((1.0 + d) * 2.0);
		double rs = 1.0 / s;

		out.setValues(c.x * rs, c.y * rs, c.z * rs, s * 0.5);
		return out;
	}
	
	static void mul(Quaternion q, Vector3 w) {
		double rx = q.w * w.x + q.y * w.z - q.z * w.y;
		double ry = q.w * w.y + q.z * w.x - q.x * w.z;
		double rz = q.w * w.z + q.x * w.y - q.y * w.x;
		double rw = -q.x * w.x - q.y * w.y - q.z * w.z;
		q.setValues(rx, ry, rz, rw);
	}
	
	static Vector3 quatRotate(Quaternion rotation, Vector3 v, Vector3 out) {
		final Quaternion q = Quaternion.copy(rotation);
	  QuaternionUtil.mul(q, v);

		final Quaternion tmp = Quaternion(0,0,0,1);
		inverse(tmp, rotation);
		q.multiply(tmp);
		
		out.setValues(q.x, q.y, q.z);
		return out;
	}
	
	static void inverse(Quaternion q, [Quaternion? src]) {
    if(src != null){
      q.x = -src.x;
      q.y = -src.y;
      q.z = -src.z;
      q.w = src.w;
    }
    else{
      q.x = -q.x;
      q.y = -q.y;
      q.z = -q.z;
    }
	}

	static void setEuler(Quaternion q, double yaw, double pitch, double roll) {
		double halfYaw = yaw * 0.5;
		double halfPitch = pitch * 0.5;
		double halfRoll = roll * 0.5;
		double cosYaw = cos(halfYaw);
		double sinYaw = sin(halfYaw);
		double cosPitch = cos(halfPitch);
		double sinPitch = sin(halfPitch);
		double cosRoll = cos(halfRoll);
		double sinRoll = sin(halfRoll);
		q.x = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
		q.y = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
		q.z = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
		q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	}
}
