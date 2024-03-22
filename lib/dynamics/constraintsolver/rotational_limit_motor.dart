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

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le�n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * Rotation limit structure for generic joints.
 * 
 * @author jezek2
 */
class RotationalLimitMotor {
	//final BulletStack stack = BulletStack.get();
	double loLimit = -BulletGlobals.simdInfinity; //!< joint limit
	double hiLimit = BulletGlobals.simdInfinity; //!< joint limit
	double targetVelocity = 0; //!< target motor velocity
	double maxMotorForce = 0.1; //!< max force on motor
	double maxLimitForce = 300.0; //!< max force on limit
	double damping = 1.0; //!< Damping.
	double limitSoftness = 0.5; //! Relaxation factor
	double erp = 0.5; //!< Error tolerance factor when joint is at limit
	double bounce = 0.0; //!< restitution factor
	bool enableMotor = false;
	
	double currentLimitError = 0;//!  How much is violated this limit
	int currentLimit = 0;//!< 0=free, 1=at lo limit, 2=at hi limit
	double accumulatedImpulse = 0;

	RotationalLimitMotor();
	
	RotationalLimitMotor.copy(RotationalLimitMotor limot) {
		targetVelocity = limot.targetVelocity;
		maxMotorForce = limot.maxMotorForce;
		limitSoftness = limot.limitSoftness;
		loLimit = limot.loLimit;
		hiLimit = limot.hiLimit;
		erp = limot.erp;
		bounce = limot.bounce;
		currentLimit = limot.currentLimit;
		currentLimitError = limot.currentLimitError;
		enableMotor = limot.enableMotor;
	}

	/**
	 * Is limited?
	 */
  bool isLimited(){
    if(loLimit>=hiLimit) return false;
    return true;
  }

	/**
	 * Need apply correction?
	 */
  bool needApplyTorques(){
    if(currentLimit == 0 && enableMotor == false) return false;
    return true;
  }

	/**
	 * Calculates error. Calculates currentLimit and currentLimitError.
	 */
	int testLimitValue(double testValue) {
		if (loLimit > hiLimit) {
			currentLimit = 0; // Free from violation
			return 0;
		}

		if (testValue < loLimit) {
			currentLimit = 1; // low limit violation
			currentLimitError = testValue - loLimit;
			return 1;
		}
		else if (testValue > hiLimit) {
			currentLimit = 2; // High limit violation
			currentLimitError = testValue - hiLimit;
			return 2;
		}

		currentLimit = 0; // Free from violation
		return 0;
	}

	/**
	 * Apply the correction impulses for two bodies.
	 */

	double solveAngularLimits(double timeStep, Vector3 axis, double jacDiagABInv, RigidBody body0, RigidBody? body1) {
		if (needApplyTorques() == false) {
			return 0.0;
		}

		double targetVelocity = this.targetVelocity;
		double maxMotorForce = this.maxMotorForce;

		// current error correction
		if (currentLimit != 0) {
			targetVelocity = -erp * currentLimitError / (timeStep);
			maxMotorForce = maxLimitForce;
		}

		maxMotorForce *= timeStep;

		// current velocity difference
		Vector3 velDiff = body0.getAngularVelocity(Vector3.zero());
		if (body1 != null) {
			velDiff.sub(body1.getAngularVelocity(Vector3.zero()));
		}

		double relVel = axis.dot(velDiff);

		// correction velocity
		double motorRelvel = limitSoftness * (targetVelocity - damping * relVel);

		if (motorRelvel < BulletGlobals.fltEpsilon && motorRelvel > -BulletGlobals.fltEpsilon) {
			return 0.0; // no need for applying force
		}

		// correction impulse
		double unclippedMotorImpulse = (1 + bounce) * motorRelvel * jacDiagABInv;

		// clip correction impulse
		double clippedMotorImpulse;

		// todo: should clip against accumulated impulse
		if (unclippedMotorImpulse > 0.0) {
			clippedMotorImpulse = unclippedMotorImpulse > maxMotorForce ? maxMotorForce : unclippedMotorImpulse;
		}
		else {
			clippedMotorImpulse = unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce : unclippedMotorImpulse;
		}

		// sort with accumulated impulses
		double lo = -1e30;
		double hi = 1e30;

		double oldaccumImpulse = accumulatedImpulse;
		double sum = oldaccumImpulse + clippedMotorImpulse;
		accumulatedImpulse = sum > hi ? 0 : sum < lo ? 0 : sum;

		clippedMotorImpulse = accumulatedImpulse - oldaccumImpulse;

		Vector3 motorImp = Vector3.zero();
		motorImp.scaleFrom(clippedMotorImpulse, axis);

		body0.applyTorqueImpulse(motorImp);
		if (body1 != null) {
			motorImp.negate();
			body1.applyTorqueImpulse(motorImp);
		}

		return clippedMotorImpulse;
	}
	
}
