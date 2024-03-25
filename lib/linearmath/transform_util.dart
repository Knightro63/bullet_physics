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
import 'package:bullet_physics/linearmath/quaternion_util.dart';
import 'package:bullet_physics/linearmath/transform.dart';

import '../core/bullet_globals.dart';
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 * Utility functions for transforms.
 * 
 * @author jezek2
 */
class TransformUtil {
	
	static const double simdSqrt12 = 0.7071067811865475244008443621048490;
	static const double angularMotionThreshold = 0.5*BulletGlobals.simdHalfPi;
	
	static double recipSqrt(double x) {
		return 1 / sqrt(x);  /* reciprocal square root */
	}

	static void planeSpace1(Vector3 n, Vector3 p, Vector3 q) {
		if (n.z.abs() > simdSqrt12) {
			// choose p in y-z plane
			double a = n.y * n.y + n.z * n.z;
			double k = recipSqrt(a);
			p.setValues(0, -n.z * k, n.y * k);
			// set q = n x p
			q.setValues(a * k, -n.x * p.z, n.x * p.y);
		}
		else {
			// choose p in x-y plane
			double a = n.x * n.x + n.y * n.y;
			double k = recipSqrt(a);
			p.setValues(-n.y * k, n.x * k, 0);
			// set q = n x p
			q.setValues(-n.z * p.y, n.z * p.x, a * k);
		}
	}
	
	static void integrateTransform(Transform? curTrans, Vector3 linvel, Vector3 angvel, double timeStep, Transform predictedTransform) {
		predictedTransform.origin.scaleAdd(timeStep, linvel, curTrans?.origin ?? Vector3.zero());
		
		Vector3 axis = Vector3.zero();
		double fAngle = angvel.length;

		// limit the angular motion
		if (fAngle * timeStep > angularMotionThreshold) {
			fAngle = angularMotionThreshold / timeStep;
		}

		if (fAngle < 0.001) {
			// use Taylor's expansions of sync function
			axis.scaleFrom(0.5 * timeStep - (timeStep * timeStep * timeStep) * (0.020833333333) * fAngle * fAngle, angvel);
		}
		else {
			// sync(fAngle) = sin(c*fAngle)/t
			axis.scaleFrom(sin(0.5 * fAngle * timeStep) / fAngle, angvel);
		}
		Quaternion dorn = Quaternion(0,0,0,0);
		dorn.setValues(axis.x, axis.y, axis.z, cos(fAngle * timeStep * 0.5));
		Quaternion? orn0 = curTrans?.getRotation(Quaternion(0,0,0,0));

		Quaternion predictedOrn = Quaternion(0,0,0,0);
		predictedOrn.multiply2(dorn, orn0 ?? Quaternion(0,0,0,0));
		predictedOrn.normalize();
//  #endif
		predictedTransform.setRotation(predictedOrn);
	}

	static void calculateVelocity(Transform transform0, Transform transform1, double timeStep, Vector3 linVel, Vector3 angVel) {
		linVel.sub2(transform1.origin,transform0.origin);
		linVel.scale(1/timeStep);

		Vector3 axis = Vector3.zero();
		List<double> angle = [];
		calculateDiffAxisAngle(transform0, transform1, axis, angle);
    angVel.scaleFrom(angle[0] / timeStep,axis);
	}
	
	static void calculateDiffAxisAngle(Transform transform0, Transform transform1, Vector3 axis, List<double> angle) {
		Matrix3 tmp = Matrix3.zero();
		tmp.setFrom(transform0.basis);
		MatrixUtil.invert(tmp);

		Matrix3 dmat = Matrix3.zero();
		dmat.multiply(transform1.basis*tmp);

		Quaternion dorn = Quaternion(0,0,0,0);
		MatrixUtil.getRotation(dmat, dorn);

		dorn.normalize();

		angle[0] = QuaternionUtil.getAngle(dorn);
		axis.setValues(dorn.x, dorn.y, dorn.z);

		// check for axis length
		double len = axis.length2;
		if (len < BulletGlobals.fltEpsilon * BulletGlobals.fltEpsilon) {
			axis.setValues(1, 0, 0);
		}
		else {
			axis.scale(1/sqrt(len));
		}
	}
}
