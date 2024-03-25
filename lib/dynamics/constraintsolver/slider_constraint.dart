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
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
 - add conversion for ODE constraint solver
*/

import "package:bullet_physics/dynamics/constraintsolver/jacobian_entry.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint_type.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 *
 * @author jezek2
 */
class SliderConstraint extends TypedConstraint {
	
	static const double scdSoftness    = 1.0;
	static const double scdDamping     = 1.0;
	static const double scdRestitution = 0.7;
	
	final Transform frameInA = Transform();
	final Transform frameInB = Transform();
	// use frameA fo define limits, if true
	bool useLinearReferenceFrameA;
	// linear limits
	double lowerLinLimit = 1;
	double upperLinLimit = -1;
	// angular limits
	double lowerAngLimit = 0;
	double upperAngLimit = 0;
	// softness, restitution and damping for different cases
	// DirLin - moving inside linear limits
	// LimLin - hitting linear limit
	// DirAng - moving inside angular limits
	// LimAng - hitting angular limit
	// OrthoLin, OrthoAng - against constraint axis
	double softnessDirLin = scdSoftness;
	double restitutionDirLin = scdRestitution;
	double dampingDirLin = 0;
	double softnessDirAng = scdSoftness;
	double restitutionDirAng = scdRestitution;
	double dampingDirAng = 0;
	double softnessLimLin = scdSoftness;
	double restitutionLimLin = scdRestitution;
	double dampingLimLin = scdDamping;
	double softnessLimAng = scdSoftness;
	double restitutionLimAng = scdRestitution;
	double dampingLimAng = scdDamping;
	double softnessOrthoLin = scdSoftness;
	double restitutionOrthoLin = scdRestitution;
	double dampingOrthoLin = scdDamping;
	double softnessOrthoAng = scdSoftness;
	double restitutionOrthoAng = scdRestitution;
	double dampingOrthoAng = scdDamping;
	
	// for interlal use
	bool solveLinLim = false;
	bool solveAngLim = false;

	List<JacobianEntry> jacLin = [JacobianEntry(), JacobianEntry(), JacobianEntry()];
	List<double> jacLinDiagABInv = [0,0,0];//double[3];

	List<JacobianEntry> jacAng = [JacobianEntry(), JacobianEntry(), JacobianEntry()];

	double timeStep = 0;
	final Transform calculatedTransformA = Transform();
	final Transform calculatedTransformB = Transform();

	final Vector3 sliderAxis = Vector3.zero();
	final Vector3 realPivotAInW = Vector3.zero();
	final Vector3 realPivotBInW = Vector3.zero();
	final Vector3 projPivotInW = Vector3.zero();
	final Vector3 delta = Vector3.zero();
	final Vector3 depth = Vector3.zero();
	final Vector3 relPosA = Vector3.zero();
	final Vector3 relPosB = Vector3.zero();

	double linPos = 0;

	double angDepth = 0;
	double kAngle = 0;

	bool poweredLinMotor = false;
	double targetLinMotorVelocity = 0;
	double maxLinMotorForce = 0;
	double accumulatedLinMotorImpulse = 0;
	
	bool poweredAngMotor = false;
	double targetAngMotorVelocity = 0;
	double maxAngMotorForce = 0;
	double accumulatedAngMotorImpulse = 0;

  SliderConstraint([RigidBody? rbA, RigidBody? rbB, Transform? frameInA, Transform? frameInB ,this.useLinearReferenceFrameA = true]):super(TypedConstraintType.slider, rbA, rbB) {
    if(frameInA != null){
      this.frameInA.copy(frameInA);
    }
    if(frameInB != null){
      this.frameInB.copy(frameInB);
    }
	}

	@override
	void buildJacobian() {
		if (useLinearReferenceFrameA) {
			buildJacobianInt(rbA, rbB, frameInA, frameInB);
		}
		else {
			buildJacobianInt(rbB, rbA, frameInB, frameInA);
		}
	}

	@override
	void solveConstraint(double timeStep) {
		this.timeStep = timeStep;
		if (useLinearReferenceFrameA) {
			solveConstraintInt(rbA, rbB);
		}
		else {
			solveConstraintInt(rbB, rbA);
		}
	}
	
	Transform getCalculatedTransformA(Transform out) {
		out.copy(calculatedTransformA);
		return out;
	}
	
	Transform getCalculatedTransformB(Transform out) {
		out.copy(calculatedTransformB);
		return out;
	}
	
	Transform getFrameOffsetA(Transform out) {
		out.copy(frameInA);
		return out;
	}

	Transform getFrameOffsetB(Transform out) {
		out.copy(frameInB);
		return out;
	}
	
	double getLowerLinLimit() {
		return lowerLinLimit;
	}

	void setLowerLinLimit(double lowerLimit) {
		lowerLinLimit = lowerLimit;
	}

	double getUpperLinLimit() {
		return upperLinLimit;
	}

	void setUpperLinLimit(double upperLimit) {
		upperLinLimit = upperLimit;
	}

	double getLowerAngLimit() {
		return lowerAngLimit;
	}

	void setLowerAngLimit(double lowerLimit) {
		lowerAngLimit = lowerLimit;
	}

	double getUpperAngLimit() {
		return upperAngLimit;
	}

	void setUpperAngLimit(double upperLimit) {
		upperAngLimit = upperLimit;
	}

	bool getUseLinearReferenceFrameA() {
		return useLinearReferenceFrameA;
	}
	
	double getSoftnessDirLin() {
		return softnessDirLin;
	}

	double getRestitutionDirLin() {
		return restitutionDirLin;
	}

	double getDampingDirLin() {
		return dampingDirLin;
	}

	double getSoftnessDirAng() {
		return softnessDirAng;
	}

	double getRestitutionDirAng() {
		return restitutionDirAng;
	}

	double getDampingDirAng() {
		return dampingDirAng;
	}

	double getSoftnessLimLin() {
		return softnessLimLin;
	}

	double getRestitutionLimLin() {
		return restitutionLimLin;
	}

	double getDampingLimLin() {
		return dampingLimLin;
	}

	double getSoftnessLimAng() {
		return softnessLimAng;
	}

	double getRestitutionLimAng() {
		return restitutionLimAng;
	}

	double getDampingLimAng() {
		return dampingLimAng;
	}

	double getSoftnessOrthoLin() {
		return softnessOrthoLin;
	}

	double getRestitutionOrthoLin() {
		return restitutionOrthoLin;
	}

	double getDampingOrthoLin() {
		return dampingOrthoLin;
	}

	double getSoftnessOrthoAng() {
		return softnessOrthoAng;
	}

	double getRestitutionOrthoAng() {
		return restitutionOrthoAng;
	}

	double getDampingOrthoAng() {
		return dampingOrthoAng;
	}
	
	void setSoftnessDirLin(double softnessDirLin) {
		this.softnessDirLin = softnessDirLin;
	}

	void setRestitutionDirLin(double restitutionDirLin) {
		this.restitutionDirLin = restitutionDirLin;
	}

	void setDampingDirLin(double dampingDirLin) {
		this.dampingDirLin = dampingDirLin;
	}

	void setSoftnessDirAng(double softnessDirAng) {
		this.softnessDirAng = softnessDirAng;
	}

	void setRestitutionDirAng(double restitutionDirAng) {
		this.restitutionDirAng = restitutionDirAng;
	}

	void setDampingDirAng(double dampingDirAng) {
		this.dampingDirAng = dampingDirAng;
	}

	void setSoftnessLimLin(double softnessLimLin) {
		this.softnessLimLin = softnessLimLin;
	}

	void setRestitutionLimLin(double restitutionLimLin) {
		this.restitutionLimLin = restitutionLimLin;
	}

	void setDampingLimLin(double dampingLimLin) {
		this.dampingLimLin = dampingLimLin;
	}

	void setSoftnessLimAng(double softnessLimAng) {
		this.softnessLimAng = softnessLimAng;
	}

	void setRestitutionLimAng(double restitutionLimAng) {
		this.restitutionLimAng = restitutionLimAng;
	}

	void setDampingLimAng(double dampingLimAng) {
		this.dampingLimAng = dampingLimAng;
	}

	void setSoftnessOrthoLin(double softnessOrthoLin) {
		this.softnessOrthoLin = softnessOrthoLin;
	}

	void setRestitutionOrthoLin(double restitutionOrthoLin) {
		this.restitutionOrthoLin = restitutionOrthoLin;
	}

	void setDampingOrthoLin(double dampingOrthoLin) {
		this.dampingOrthoLin = dampingOrthoLin;
	}

	void setSoftnessOrthoAng(double softnessOrthoAng) {
		this.softnessOrthoAng = softnessOrthoAng;
	}

	void setRestitutionOrthoAng(double restitutionOrthoAng) {
		this.restitutionOrthoAng = restitutionOrthoAng;
	}

	void setDampingOrthoAng(double dampingOrthoAng) {
		this.dampingOrthoAng = dampingOrthoAng;
	}

	void setPoweredLinMotor(bool onOff) {
		poweredLinMotor = onOff;
	}

	bool getPoweredLinMotor() {
		return poweredLinMotor;
	}

	void setTargetLinMotorVelocity(double targetLinMotorVelocity) {
		this.targetLinMotorVelocity = targetLinMotorVelocity;
	}

	double getTargetLinMotorVelocity() {
		return targetLinMotorVelocity;
	}

	void setMaxLinMotorForce(double maxLinMotorForce) {
		this.maxLinMotorForce = maxLinMotorForce;
	}

	double getMaxLinMotorForce() {
		return maxLinMotorForce;
	}

	void setPoweredAngMotor(bool onOff) {
		poweredAngMotor = onOff;
	}

	bool getPoweredAngMotor() {
		return poweredAngMotor;
	}

	void setTargetAngMotorVelocity(double targetAngMotorVelocity) {
		this.targetAngMotorVelocity = targetAngMotorVelocity;
	}

	double getTargetAngMotorVelocity() {
		return targetAngMotorVelocity;
	}

	void setMaxAngMotorForce(double maxAngMotorForce) {
		this.maxAngMotorForce = maxAngMotorForce;
	}

	double getMaxAngMotorForce() {
		return maxAngMotorForce;
	}

	double getLinearPos() {
		return linPos;
	}

	// access for ODE solver

	bool getSolveLinLimit() {
		return solveLinLim;
	}

	double getLinDepth() {
		return depth.x;
	}

	bool getSolveAngLimit() {
		return solveAngLim;
	}

	double getAngDepth() {
		return angDepth;
	}
	
	// internal
	
	void buildJacobianInt(RigidBody rbA, RigidBody rbB, Transform frameInA, Transform frameInB) {
		Transform tmpTrans = Transform();
		Transform tmpTrans1 = Transform();
		Transform tmpTrans2 = Transform();
		Vector3 tmp = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		// calculate transforms
		calculatedTransformA.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
		calculatedTransformB.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
		realPivotAInW.setFrom(calculatedTransformA.origin);
		realPivotBInW.setFrom(calculatedTransformB.origin);
		calculatedTransformA.basis.getColumnWith(0, tmp);
		sliderAxis.setFrom(tmp); // aint X
		delta.sub2(realPivotBInW, realPivotAInW);
		projPivotInW.scaleAdd(sliderAxis.dot(delta), sliderAxis, realPivotAInW);
		relPosA.sub2(projPivotInW, rbA.getCenterOfMassPosition(tmp));
		relPosB.sub2(realPivotBInW, rbB.getCenterOfMassPosition(tmp));
		Vector3 normalWorld = Vector3.zero();

		// linear part
		for (int i=0; i<3; i++) {
			calculatedTransformA.basis.getColumnWith(i, normalWorld);

			Matrix3 mat1 = rbA.getCenterOfMassTransform(tmpTrans1).basis;
			mat1.transpose();

			Matrix3 mat2 = rbB.getCenterOfMassTransform(tmpTrans2).basis;
			mat2.transpose();

			jacLin[i].initAll(
					mat1,
					mat2,
					relPosA,
					relPosB,
					normalWorld,
					rbA.getInvInertiaDiagLocal(tmp),
					rbA.getInvMass(),
					rbB.getInvInertiaDiagLocal(tmp2),
					rbB.getInvMass());
			jacLinDiagABInv[i] = 1 / jacLin[i].getDiagonal();
			VectorUtil.setCoord(depth, i, delta.dot(normalWorld));
		}
		testLinLimits();

		// angular part
		for (int i=0; i<3; i++) {
			calculatedTransformA.basis.getColumnWith(i, normalWorld);

			Matrix3 mat1 = rbA.getCenterOfMassTransform(tmpTrans1).basis;
			mat1.transpose();

			Matrix3 mat2 = rbB.getCenterOfMassTransform(tmpTrans2).basis;
			mat2.transpose();

			jacAng[i].initMultiWorld(
					normalWorld,
					mat1,
					mat2,
					rbA.getInvInertiaDiagLocal(tmp),
					rbB.getInvInertiaDiagLocal(tmp2));
		}
		testAngLimits();

		Vector3 axisA = Vector3.zero();
		calculatedTransformA.basis.getColumnWith(0, axisA);
		kAngle = 1 / (rbA.computeAngularImpulseDenominator(axisA) + rbB.computeAngularImpulseDenominator(axisA));
		// clear accumulator for motors
		accumulatedLinMotorImpulse = 0;
		accumulatedAngMotorImpulse = 0;
	}
	
	void solveConstraintInt(RigidBody rbA, RigidBody rbB) {
		Vector3 tmp = Vector3.zero();

		// linear
		Vector3 velA = rbA.getVelocityInLocalPoint(relPosA, Vector3.zero());
		Vector3 velB = rbB.getVelocityInLocalPoint(relPosB, Vector3.zero());
		Vector3 vel = Vector3.zero();
		vel.sub2(velA, velB);

		Vector3 impulseVector = Vector3.zero();

		for (int i=0; i<3; i++) {
			Vector3 normal = jacLin[i].linearJointAxis;
			double relVel = normal.dot(vel);
			// calculate positional error
			double depth = VectorUtil.getCoord(this.depth, i);
			// get parameters
			double softness = (i != 0)? softnessOrthoLin : (solveLinLim? softnessLimLin : softnessDirLin);
			double restitution = (i != 0)? restitutionOrthoLin : (solveLinLim? restitutionLimLin : restitutionDirLin);
			double damping = (i != 0)? dampingOrthoLin : (solveLinLim? dampingLimLin : dampingDirLin);
			// calcutate and apply impulse
			double normalImpulse = softness * (restitution * depth / timeStep - damping * relVel) * jacLinDiagABInv[i];
			impulseVector.scaleFrom(normalImpulse, normal);
			rbA.applyImpulse(impulseVector, relPosA);
			tmp.negateFrom(impulseVector);
			rbB.applyImpulse(tmp, relPosB);

			if (poweredLinMotor && (i == 0)) {
				// apply linear motor
				if (accumulatedLinMotorImpulse < maxLinMotorForce) {
					double desiredMotorVel = targetLinMotorVelocity;
					double motorRelvel = desiredMotorVel + relVel;
					normalImpulse = -motorRelvel * jacLinDiagABInv[i];
					// clamp accumulated impulse
					double newAcc = accumulatedLinMotorImpulse + normalImpulse.abs();
					if (newAcc > maxLinMotorForce) {
						newAcc = maxLinMotorForce;
					}
					double del = newAcc - accumulatedLinMotorImpulse;
					if (normalImpulse < 0) {
						normalImpulse = -del;
					}
					else {
						normalImpulse = del;
					}
					accumulatedLinMotorImpulse = newAcc;
					// apply clamped impulse
					impulseVector.scaleFrom(normalImpulse, normal);
					rbA.applyImpulse(impulseVector, relPosA);
					tmp.negateFrom(impulseVector);
					rbB.applyImpulse(tmp, relPosB);
				}
			}
		}

		// angular
		// get axes in world space
		Vector3 axisA = Vector3.zero();
		calculatedTransformA.basis.getColumnWith(0, axisA);
		Vector3 axisB = Vector3.zero();
		calculatedTransformB.basis.getColumnWith(0, axisB);

		Vector3 angVelA = rbA.getAngularVelocity(Vector3.zero());
		Vector3 angVelB = rbB.getAngularVelocity(Vector3.zero());

		Vector3 angVelAroundAxisA = Vector3.zero();
		angVelAroundAxisA.scaleFrom(axisA.dot(angVelA), axisA);
		Vector3 angVelAroundAxisB = Vector3.zero();
		angVelAroundAxisB.scaleFrom(axisB.dot(angVelB), axisB);

		Vector3 angAorthog = Vector3.zero();
		angAorthog.sub2(angVelA, angVelAroundAxisA);
		Vector3 angBorthog = Vector3.zero();
		angBorthog.sub2(angVelB, angVelAroundAxisB);
		Vector3 velrelOrthog = Vector3.zero();
		velrelOrthog.sub2(angAorthog, angBorthog);

		// solve orthogonal angular velocity correction
		double len = velrelOrthog.length;
		if (len > 0.00001) {
			Vector3 normal = Vector3.zero();
			normal.normalizeFrom(velrelOrthog);
			double denom = rbA.computeAngularImpulseDenominator(normal) + rbB.computeAngularImpulseDenominator(normal);
			velrelOrthog.scale((1 / denom) * dampingOrthoAng * softnessOrthoAng);
		}

		// solve angular positional correction
		Vector3 angularError = Vector3.zero();
		angularError.cross2(axisA, axisB);
		angularError.scale(1 / timeStep);
		double len2 = angularError.length;
		if (len2 > 0.00001) {
			Vector3 normal2 = Vector3.zero();
			normal2.normalizeFrom(angularError);
			double denom2 = rbA.computeAngularImpulseDenominator(normal2) + rbB.computeAngularImpulseDenominator(normal2);
			angularError.scale((1 / denom2) * restitutionOrthoAng * softnessOrthoAng);
		}

		// apply impulse
		tmp.negateFrom(velrelOrthog);
		tmp.add(angularError);
		rbA.applyTorqueImpulse(tmp);
		tmp.sub2(velrelOrthog, angularError);
		rbB.applyTorqueImpulse(tmp);
		double impulseMag;

		// solve angular limits
		if (solveAngLim) {
			tmp.sub2(angVelB, angVelA);
			impulseMag = tmp.dot(axisA) * dampingLimAng + angDepth * restitutionLimAng / timeStep;
			impulseMag *= kAngle * softnessLimAng;
		}
		else {
			tmp.sub2(angVelB, angVelA);
			impulseMag = tmp.dot(axisA) * dampingDirAng + angDepth * restitutionDirAng / timeStep;
			impulseMag *= kAngle * softnessDirAng;
		}
		Vector3 impulse = Vector3.zero();
		impulse.scaleFrom(impulseMag, axisA);
		rbA.applyTorqueImpulse(impulse);
		tmp.negateFrom(impulse);
		rbB.applyTorqueImpulse(tmp);

		// apply angular motor
		if (poweredAngMotor) {
			if (accumulatedAngMotorImpulse < maxAngMotorForce) {
				Vector3 velrel = Vector3.zero();
				velrel.sub2(angVelAroundAxisA, angVelAroundAxisB);
				double projRelVel = velrel.dot(axisA);

				double desiredMotorVel = targetAngMotorVelocity;
				double motorRelvel = desiredMotorVel - projRelVel;

				double angImpulse = kAngle * motorRelvel;
				// clamp accumulated impulse
				double newAcc = accumulatedAngMotorImpulse + angImpulse.abs();
				if (newAcc > maxAngMotorForce) {
					newAcc = maxAngMotorForce;
				}
				double del = newAcc - accumulatedAngMotorImpulse;
				if (angImpulse < 0) {
					angImpulse = -del;
				} else {
					angImpulse = del;
				}
				accumulatedAngMotorImpulse = newAcc;

				// apply clamped impulse
				Vector3 motorImp = Vector3.zero();
				motorImp.scaleFrom(angImpulse, axisA);
				rbA.applyTorqueImpulse(motorImp);
				tmp.negateFrom(motorImp);
				rbB.applyTorqueImpulse(tmp);
			}
		}
	}
	
	// shared code used by ODE solver
	
	void calculateTransforms() {
		Transform tmpTrans = Transform();

		if (useLinearReferenceFrameA) {
			calculatedTransformA.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
			calculatedTransformB.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
		}
		else {
			calculatedTransformA.mul(rbB.getCenterOfMassTransform(tmpTrans), frameInB);
			calculatedTransformB.mul(rbA.getCenterOfMassTransform(tmpTrans), frameInA);
		}
		realPivotAInW.setFrom(calculatedTransformA.origin);
		realPivotBInW.setFrom(calculatedTransformB.origin);
		calculatedTransformA.basis.getColumnWith(0, sliderAxis); // aint X
		delta.sub2(realPivotBInW, realPivotAInW);
		projPivotInW.scaleAdd(sliderAxis.dot(delta), sliderAxis, realPivotAInW);
		Vector3 normalWorld = Vector3.zero();
		// linear part
		for (int i=0; i<3; i++) {
			calculatedTransformA.basis.getColumnWith(i, normalWorld);
			VectorUtil.setCoord(depth, i, delta.dot(normalWorld));
		}
	}

	void testLinLimits() {
		solveLinLim = false;
		linPos = depth.x;
		if (lowerLinLimit <= upperLinLimit) {
			if (depth.x > upperLinLimit) {
				depth.x -= upperLinLimit;
				solveLinLim = true;
			}
			else if (depth.x < lowerLinLimit) {
				depth.x -= lowerLinLimit;
				solveLinLim = true;
			}
			else {
				depth.x = 0;
			}
		}
		else {
			depth.x = 0;
		}
	}
	
	void testAngLimits() {
		angDepth = 0;
		solveAngLim = false;
		if (lowerAngLimit <= upperAngLimit) {
			Vector3 axisA0 = Vector3.zero();
			calculatedTransformA.basis.getColumnWith(1, axisA0);
			Vector3 axisA1 = Vector3.zero();
			calculatedTransformA.basis.getColumnWith(2, axisA1);
			Vector3 axisB0 = Vector3.zero();
			calculatedTransformB.basis.getColumnWith(1, axisB0);

			double rot = atan2(axisB0.dot(axisA1), axisB0.dot(axisA0));
			if (rot < lowerAngLimit) {
				angDepth = rot - lowerAngLimit;
				solveAngLim = true;
			}
			else if (rot > upperAngLimit) {
				angDepth = rot - upperAngLimit;
				solveAngLim = true;
			}
		}
	}
	
	// access for PE Solver
	
	Vector3 getAncorInA(Vector3 out) {
		Transform tmpTrans = Transform();

		Vector3 ancorInA = out;
		ancorInA.scaleAdd((lowerLinLimit + upperLinLimit) * 0.5, sliderAxis, realPivotAInW);
		rbA.getCenterOfMassTransform(tmpTrans);
		tmpTrans.inverse();
		tmpTrans.transform(ancorInA);
		return ancorInA;
	}

	Vector3 getAncorInB(Vector3 out) {
		Vector3 ancorInB = out;
		ancorInB.setFrom(frameInB.origin);
		return ancorInB;
	}

}
