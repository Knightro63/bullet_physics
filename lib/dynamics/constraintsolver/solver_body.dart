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

import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * SolverBody is an internal data structure for the constraint solver. Only necessary
 * data is packed to increase cache coherence/performance.
 * 
 * @author jezek2
 */
class SolverBody {
	//final BulletStack stack = BulletStack.get();
	final Vector3 angularVelocity = Vector3.zero();
	double angularFactor = 0;
	double invMass = 0;
	double friction = 0;
	RigidBody? originalBody;
	final Vector3 linearVelocity = Vector3.zero();
	final Vector3 centerOfMassPosition = Vector3.zero();

	final Vector3 pushVelocity = Vector3.zero();
	final Vector3 turnVelocity = Vector3.zero();
	
	void getVelocityInLocalPoint(Vector3 relPos, Vector3 velocity) {
		Vector3 tmp = Vector3.zero();
		tmp.cross2(angularVelocity, relPos);
		velocity.add2(linearVelocity, tmp);
	}

	/**
	 * Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position.
	 */
	void internalApplyImpulse(Vector3 linearComponent, Vector3 angularComponent, double impulseMagnitude) {
		if (invMass != 0) {
			linearVelocity.scaleAdd(impulseMagnitude, linearComponent, linearVelocity);
			angularVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, angularVelocity);
		}
	}

	void internalApplyPushImpulse(Vector3 linearComponent, Vector3 angularComponent, double impulseMagnitude) {
		if (invMass != 0) {
			pushVelocity.scaleAdd(impulseMagnitude, linearComponent, pushVelocity);
			turnVelocity.scaleAdd(impulseMagnitude * angularFactor, angularComponent, turnVelocity);
		}
	}

	void writebackVelocity([double? timeStep]) {
		if (invMass != 0) {
			originalBody?.setLinearVelocity(linearVelocity);
			originalBody?.setAngularVelocity(angularVelocity);

			// correct the position/orientation based on push/turn recovery
      if(timeStep != null){
        Transform newTransform = Transform();
        Transform? curTrans = originalBody?.getWorldTransform(Transform());
        TransformUtil.integrateTransform(curTrans, pushVelocity, turnVelocity, timeStep, newTransform);
        originalBody?.setWorldTransform(newTransform);
      }
		}
	}
	
	void readVelocity() {
		if (invMass != 0) {
			originalBody?.getLinearVelocity(linearVelocity);
			originalBody?.getAngularVelocity(angularVelocity);
		}
	}
	
}
