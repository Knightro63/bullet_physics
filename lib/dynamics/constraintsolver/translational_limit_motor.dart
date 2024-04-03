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
 *
 * @author jezek2
 */
class TranslationalLimitMotor {
	//final BulletStack stack = BulletStack.get();
	
	final Vector3 lowerLimit = Vector3.zero(); //!< the constraint lower limits
	final Vector3 upperLimit = Vector3.zero(); //!< the constraint upper limits
	final Vector3 accumulatedImpulse = Vector3.zero();
	
	double limitSoftness = 0.7;     //!< Softness for linear limit
	double damping       = 1.0;           //!< Damping for linear limit
	double restitution   = 0.5;       //!< Bounce parameter for linear limit

  // added for 6dofSpring
  final List<bool> enableMotor    = [false,false,false];
  final Vector3 targetVelocity    = Vector3.zero();   //!< target motor velocity
	final Vector3 maxMotorForce     = Vector3.zero();   //!< max force on motor
  final Vector3 maxLimitForce     = Vector3.zero();   //!< max force on limit
  final Vector3 currentLimitError = Vector3.zero();   //!  How much is violated this limit
  final Vector3 currentLinearDiff = Vector3.zero();   //!  Current relative offset of constraint frames
  final List<int> currentLimit    = [0,0,0];       //!< 0=free, 1=at lower limit, 2=at upper limit
        

	TranslationalLimitMotor() {
		lowerLimit.setValues(0, 0, 0);
		upperLimit.setValues(0, 0, 0);
		accumulatedImpulse.setValues(0, 0, 0);
    targetVelocity.setValues(0, 0, 0);
    maxMotorForce.setValues(0.1, 0.1, 0.1);
    maxLimitForce.setValues(300.0, 300.0, 300.0);
	}


	TranslationalLimitMotor.copy(TranslationalLimitMotor other) {
		lowerLimit.setFrom(other.lowerLimit);
		upperLimit.setFrom(other.upperLimit);
		accumulatedImpulse.setFrom(other.accumulatedImpulse);
		limitSoftness = other.limitSoftness;
		damping = other.damping;
		restitution = other.restitution;
	}

	/**
	 * Test limit.<p>
	 * - free means upper &lt; lower,<br>
	 * - locked means upper == lower<br>
	 * - limited means upper &gt; lower<br>
	 * - limitIndex: first 3 are linear, next 3 are angular
	 */
	bool isLimited(int limitIndex) {
		return (VectorUtil.getCoord(upperLimit, limitIndex) >= VectorUtil.getCoord(lowerLimit, limitIndex));
	}

	/**
	 * Need apply correction?
	 */
  bool needApplyForces(int idx){
    if(currentLimit[idx] == 0 && enableMotor[idx] == false) {
      return false;
    } 
    return true;
  }

  int testLimitValue(int limitIndex, double testValue){
    double loLimit = VectorUtil.getCoord(lowerLimit, limitIndex);
    double hiLimit = VectorUtil.getCoord(upperLimit, limitIndex);
    if(loLimit > hiLimit){
      currentLimit[limitIndex] = 0;//Free from violation
      VectorUtil.setCoord(currentLimitError, limitIndex, 0);
      return 0;
    }

    if (testValue < loLimit){
      currentLimit[limitIndex] = 2;//low limit violation
      VectorUtil.setCoord(currentLimitError, limitIndex, testValue - loLimit);
      return 2;
    }
    else if (testValue > hiLimit){
      currentLimit[limitIndex] = 1;//High limit violation
      VectorUtil.setCoord(currentLimitError, limitIndex, testValue - hiLimit);
      return 1;
    }

    currentLimit[limitIndex] = 0;//Free from violation
    VectorUtil.setCoord(currentLimitError, limitIndex, 0);
    return 0;
  }

	double solveLinearAxis(double timeStep, double jacDiagABInv, RigidBody body1, Vector3 pointInA, RigidBody body2, Vector3 pointInB, int limitIndex, Vector3 axisNormalOnA, Vector3 anchorPos) {
		Vector3 tmp = Vector3.zero();
		Vector3 tmpVec = Vector3.zero();
		
		// find relative velocity
		Vector3 relPos1 = Vector3.zero();
		//relPos1.sub(pointInA, body1.getCenterOfMassPosition(tmpVec));
		relPos1.sub2(anchorPos, body1.getCenterOfMassPosition(tmpVec));

		Vector3 relPos2 = Vector3.zero();
		//relPos2.sub(pointInB, body2.getCenterOfMassPosition(tmpVec));
		relPos2.sub2(anchorPos, body2.getCenterOfMassPosition(tmpVec));

		Vector3 vel1 = body1.getVelocityInLocalPoint(relPos1, Vector3.zero());
		Vector3 vel2 = body2.getVelocityInLocalPoint(relPos2, Vector3.zero());
		Vector3 vel = Vector3.zero();
		vel.sub2(vel1, vel2);

		double relVel = axisNormalOnA.dot(vel);

		// apply displacement correction
    double targetVelocity   = VectorUtil.getCoord(this.targetVelocity, limitIndex);
    double maxMotorForce    = VectorUtil.getCoord(this.maxMotorForce, limitIndex);

    double limErr = VectorUtil.getCoord(currentLimitError, limitIndex);
    if (currentLimit[limitIndex] != 0){
        targetVelocity = restitution * limErr / (timeStep);
        maxMotorForce = VectorUtil.getCoord(maxLimitForce, limitIndex);
    }
		maxMotorForce *= timeStep;


    // correction velocity
		double motorRelvel = limitSoftness * (targetVelocity - damping * relVel);
		if (motorRelvel < BulletGlobals.fltEpsilon && motorRelvel > -BulletGlobals.fltEpsilon) {
			return 0.0; // no need for applying force
		}
                
                // correction impulse
		double unclippedMotorImpulse = motorRelvel * jacDiagABInv;

		// clip correction impulse
		double clippedMotorImpulse;

		// todo: should clip against accumulated impulse
		if (unclippedMotorImpulse > 0.0) {
			clippedMotorImpulse = unclippedMotorImpulse > maxMotorForce ? maxMotorForce : unclippedMotorImpulse;
		}
		else {
			clippedMotorImpulse = unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce : unclippedMotorImpulse;
		}

    double normalImpulse = clippedMotorImpulse;

     // sort with accumulated impulses
		double lo = -1e30;
		double hi = 1e30;
    
		double oldNormalImpulse = VectorUtil.getCoord(accumulatedImpulse, limitIndex);
		double sum = oldNormalImpulse + normalImpulse;
		VectorUtil.setCoord(accumulatedImpulse, limitIndex, sum > hi ? 0 : sum < lo ? 0 : sum);
		normalImpulse = VectorUtil.getCoord(accumulatedImpulse, limitIndex) - oldNormalImpulse;

		Vector3 impulseVector = Vector3.zero();
		impulseVector.scaleFrom(normalImpulse, axisNormalOnA);
		body1.applyImpulse(impulseVector, relPos1);

		tmp.negateFrom(impulseVector);
		body2.applyImpulse(tmp, relPos2);
		return normalImpulse;
	}
	
}
