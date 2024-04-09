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

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/dynamics/constraintsolver/jacobian_entry.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint_type.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/quaternion_util.dart";
import "package:bullet_physics/linearmath/scalar_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 * Hinge constraint between two rigid bodies each with a pivot point that descibes
 * the axis location in local space. Axis defines the orientation of the hinge axis.
 * 
 * @author jezek2
 */
class HingeConstraint extends TypedConstraint {

	List<JacobianEntry> _jac/*[3]*/ = [JacobianEntry(), JacobianEntry(), JacobianEntry()]; // 3 orthogonal linear constraints
	List<JacobianEntry> _jacAng/*[3]*/ = [JacobianEntry(), JacobianEntry(), JacobianEntry()]; // 2 orthogonal angular constraints+ 1 for limit/motor

	final Transform _rbAFrame = Transform(); // constraint axii. Assumes z is hinge axis.
	final Transform _rbBFrame = Transform();

	double _motorTargetVelocity = 0;
	double _maxMotorImpulse = 0;

	double _limitSoftness = 0.9; 
	double _biasFactor = 0.3; 
	double _relaxationFactor = 1.0; 

	double _lowerLimit = 1e30;	
	double _upperLimit = -1e30;	
	
	double _kHinge = 0;

	double _limitSign = 0;
	double _correction = 0;

	double _accLimitImpulse = 0;

	bool _angularOnly = false;
	bool _enableAngularMotor = false;
	bool _solveLimit = false;

	HingeConstraint.multiBody(RigidBody rbA, RigidBody rbB, Vector3 pivotInA, Vector3 pivotInB, Vector3 axisInA, Vector3 axisInB):super(TypedConstraintType.hinge, rbA, rbB){
		_rbAFrame.origin.setFrom(pivotInA);

		// since no frame is given, assume this to be zero angle and just pick rb transform axis
		Vector3 rbAxisA1 = Vector3.zero();
		Vector3 rbAxisA2 = Vector3.zero();
		
		Transform centerOfMassA = rbA.getCenterOfMassTransform(Transform());
		centerOfMassA.basis.getColumnWith(0, rbAxisA1);
		double projection = axisInA.dot(rbAxisA1);

		if (projection >= 1.0 - BulletGlobals.simdEpsilon) {
			centerOfMassA.basis.getColumnWith(2, rbAxisA1);
			rbAxisA1.negate();
			centerOfMassA.basis.getColumnWith(1, rbAxisA2);
		} 
    else if (projection <= -1.0 + BulletGlobals.simdEpsilon) {           
			centerOfMassA.basis.getColumnWith(2, rbAxisA1);                            
			centerOfMassA.basis.getColumnWith(1, rbAxisA2);
		} 
    else {
			rbAxisA2.cross2(axisInA, rbAxisA1);                                                                
			rbAxisA1.cross2(rbAxisA2, axisInA);                                                                                            
		}

		_rbAFrame.basis.setRowByValues(0, rbAxisA1.x, rbAxisA2.x, axisInA.x);
		_rbAFrame.basis.setRowByValues(1, rbAxisA1.y, rbAxisA2.y, axisInA.y);
		_rbAFrame.basis.setRowByValues(2, rbAxisA1.z, rbAxisA2.z, axisInA.z);

		Quaternion rotationArc = QuaternionUtil.intestArcQuat(axisInA, axisInB, Quaternion(0,0,0,0));
		Vector3 rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, Vector3.zero());
		Vector3 rbAxisB2 = Vector3.zero();
		rbAxisB2.cross2(axisInB, rbAxisB1);

		_rbBFrame.origin.setFrom(pivotInB);
		_rbBFrame.basis.setRowByValues(0, rbAxisB1.x, rbAxisB2.x, -axisInB.x);
		_rbBFrame.basis.setRowByValues(1, rbAxisB1.y, rbAxisB2.y, -axisInB.y);
		_rbBFrame.basis.setRowByValues(2, rbAxisB1.z, rbAxisB2.z, -axisInB.z);			
	}

	HingeConstraint.singleBody(RigidBody rbA, Vector3 pivotInA, Vector3 axisInA):super(TypedConstraintType.hinge, rbA){
		// since no frame is given, assume this to be zero angle and just pick rb transform axis
		// fixed axis in worldspace
		Vector3 rbAxisA1 = Vector3.zero();
		Transform centerOfMassA = rbA.getCenterOfMassTransform(Transform());
		centerOfMassA.basis.getColumnWith(0, rbAxisA1);

		double projection = rbAxisA1.dot(axisInA);
		if (projection > BulletGlobals.fltEpsilon) {
			rbAxisA1.scale(projection);
			rbAxisA1.sub(axisInA);
		}
		else {
			centerOfMassA.basis.getColumnWith(1, rbAxisA1);
		}

		Vector3 rbAxisA2 = Vector3.zero();
		rbAxisA2.cross2(axisInA, rbAxisA1);

		_rbAFrame.origin.setFrom(pivotInA);
		_rbAFrame.basis.setRowByValues(0, rbAxisA1.x, rbAxisA2.x, axisInA.x);
		_rbAFrame.basis.setRowByValues(1, rbAxisA1.y, rbAxisA2.y, axisInA.y);
		_rbAFrame.basis.setRowByValues(2, rbAxisA1.z, rbAxisA2.z, axisInA.z);

		Vector3 axisInB = Vector3.zero();
		axisInB.negateFrom(axisInA);
		centerOfMassA.basis.transform(axisInB);

		Quaternion rotationArc = QuaternionUtil.intestArcQuat(axisInA, axisInB, Quaternion(0,0,0,0));
		Vector3 rbAxisB1 = QuaternionUtil.quatRotate(rotationArc, rbAxisA1, Vector3.zero());
		Vector3 rbAxisB2 = Vector3.zero();
		rbAxisB2.cross2(axisInB, rbAxisB1);

		_rbBFrame.origin.setFrom(pivotInA);
		centerOfMassA.transform(_rbBFrame.origin);
		_rbBFrame.basis.setRowByValues(0, rbAxisB1.x, rbAxisB2.x, axisInB.x);
		_rbBFrame.basis.setRowByValues(1, rbAxisB1.y, rbAxisB2.y, axisInB.y);
		_rbBFrame.basis.setRowByValues(2, rbAxisB1.z, rbAxisB2.z, axisInB.z);
	}

	HingeConstraint([RigidBody? rbA,  Transform? rbAFrame, RigidBody? rbB, Transform? rbBFrame]):super(TypedConstraintType.hinge, rbA, rbB) {
    if(rbA != null && rbAFrame != null){
      _rbAFrame.copy(rbAFrame);
      _rbBFrame.basis.storage[2] *= -1;
      _rbBFrame.basis.storage[5] *= -1;
      _rbBFrame.basis.storage[8] *= -1;

      if(rbB == null && rbBFrame == null){
        _rbBFrame.copy(rbAFrame);
        _rbBFrame.origin.setFrom(_rbAFrame.origin);
        rbA.getCenterOfMassTransform(Transform()).transform(_rbBFrame.origin);
      }
      else{
        _rbBFrame.copy(rbBFrame!);
      }
    }
	}
	
	@override
	void buildJacobian() {
		Vector3 tmp = Vector3.zero();
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();
		Vector3 tmpVec = Vector3.zero();
		Matrix3 mat1 = Matrix3.zero();
		Matrix3 mat2 = Matrix3.zero();
		
		Transform centerOfMassA = rbA.getCenterOfMassTransform(Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(Transform());

		appliedImpulse = 0;

		if (!_angularOnly) {
			Vector3 pivotAInW = Vector3.copy(_rbAFrame.origin);
			centerOfMassA.transform(pivotAInW);

			Vector3 pivotBInW = Vector3.copy(_rbBFrame.origin);
			centerOfMassB.transform(pivotBInW);

			Vector3 relPos = Vector3.zero();
			relPos.sub2(pivotBInW, pivotAInW);

			List<Vector3> normal/*[3]*/ = [Vector3.zero(), Vector3.zero(), Vector3.zero()];
			if (relPos.length2 > BulletGlobals.fltEpsilon) {
				normal[0].setFrom(relPos);
				normal[0].normalize();
			}
			else {
				normal[0].setValues(1, 0, 0);
			}

			TransformUtil.planeSpace1(normal[0], normal[1], normal[2]);

			for (int i = 0; i < 3; i++) {
				mat1.transposeFrom(centerOfMassA.basis);
				mat2.transposeFrom(centerOfMassB.basis);

				tmp1.sub2(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
				tmp2.sub2(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

				_jac[i].initAll(
          mat1,
          mat2,
          tmp1,
          tmp2,
          normal[i],
          rbA.getInvInertiaDiagLocal(Vector3.zero()),
          rbA.getInvMass(),
          rbB.getInvInertiaDiagLocal(Vector3.zero()),
          rbB.getInvMass()
        );
			}
		}

		// calculate two perpendicular jointAxis, orthogonal to hingeAxis
		// these two jointAxis require equal angular velocities for both bodies

		// this is unused for now, it's a todo
		Vector3 jointAxis0local = Vector3.zero();
		Vector3 jointAxis1local = Vector3.zero();

		_rbAFrame.basis.getColumnWith(2, tmp);
		TransformUtil.planeSpace1(tmp, jointAxis0local, jointAxis1local);

		// TODO: check this
		//getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);

		Vector3 jointAxis0 = Vector3.copy(jointAxis0local);
		centerOfMassA.basis.transform(jointAxis0);

		Vector3 jointAxis1 = Vector3.copy(jointAxis1local);
		centerOfMassA.basis.transform(jointAxis1);

		Vector3 hingeAxisWorld = Vector3.zero();
		_rbAFrame.basis.getColumnWith(2, hingeAxisWorld);
		centerOfMassA.basis.transform(hingeAxisWorld);

		mat1.transposeFrom(centerOfMassA.basis);
		mat2.transposeFrom(centerOfMassB.basis);
		_jacAng[0].initMultiWorld(
      jointAxis0,
			mat1,
			mat2,
			rbA.getInvInertiaDiagLocal(Vector3.zero()),
			rbB.getInvInertiaDiagLocal(Vector3.zero())
    );

		// JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
		_jacAng[1].initMultiWorld(
      jointAxis1,
      mat1,
      mat2,
      rbA.getInvInertiaDiagLocal(Vector3.zero()),
      rbB.getInvInertiaDiagLocal(Vector3.zero())
    );

		// JAVA NOTE: reused mat1 and mat2, as recomputation is not needed
		_jacAng[2].initMultiWorld(
      hingeAxisWorld,
      mat1,
      mat2,
      rbA.getInvInertiaDiagLocal(Vector3.zero()),
      rbB.getInvInertiaDiagLocal(Vector3.zero())
    );

		// Compute limit information
		double hingeAngle = getHingeAngle();

		//set bias, sign, clear accumulator
		_correction = 0;
		_limitSign = 0;
		_solveLimit = false;
		_accLimitImpulse = 0;

		if (_lowerLimit < _upperLimit) {
			if (hingeAngle <= _lowerLimit * _limitSoftness) {
				_correction = (_lowerLimit - hingeAngle);
				_limitSign = 1.0;
				_solveLimit = true;
			}
			else if (hingeAngle >= _upperLimit * _limitSoftness) {
				_correction = _upperLimit - hingeAngle;
				_limitSign = -1.0;
				_solveLimit = true;
			}
		}

		// Compute K = J*W*J' for hinge axis
		Vector3 axisA = Vector3.zero();
		_rbAFrame.basis.getColumnWith(2, axisA);
		centerOfMassA.basis.transform(axisA);

		_kHinge = 1.0 / (getRigidBodyA().computeAngularImpulseDenominator(axisA) + getRigidBodyB().computeAngularImpulseDenominator(axisA));
	}

	@override
	void solveConstraint(double timeStep) {
		Vector3 tmp = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();
		Vector3 tmpVec = Vector3.zero();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(Transform());
		
		Vector3 pivotAInW = Vector3.copy(_rbAFrame.origin);
		centerOfMassA.transform(pivotAInW);

		Vector3 pivotBInW = Vector3.copy(_rbBFrame.origin);
		centerOfMassB.transform(pivotBInW);

		double tau = 0.3;

		// linear part
		if (!_angularOnly) {
			Vector3 relPos1 = Vector3.zero();
			relPos1.sub2(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));

			Vector3 relPos2 = Vector3.zero();
			relPos2.sub2(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

			Vector3 vel1 = rbA.getVelocityInLocalPoint(relPos1, Vector3.zero());
			Vector3 vel2 = rbB.getVelocityInLocalPoint(relPos2, Vector3.zero());
			Vector3 vel = Vector3.zero();
			vel.sub2(vel1, vel2);

			for (int i = 0; i < 3; i++) {
				Vector3 normal = _jac[i].linearJointAxis;
				double jacDiagABInv = 1 / _jac[i].getDiagonal();

				double relVel;
				relVel = normal.dot(vel);
				// positional error (zeroth order error)
				tmp.sub2(pivotAInW, pivotBInW);
				double depth = -(tmp).dot(normal); // this is the error projected on the normal
				double impulse = depth * tau / timeStep * jacDiagABInv - relVel * jacDiagABInv;
				appliedImpulse += impulse;
				Vector3 impulseVector = Vector3.zero();
				impulseVector.scaleFrom(impulse, normal);

				tmp.sub2(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
				rbA.applyImpulse(impulseVector, tmp);

				tmp.negateFrom(impulseVector);
				tmp2.sub2(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
				rbB.applyImpulse(tmp, tmp2);
			}
		}


		{
			// solve angular part

			// get axes in world space
			Vector3 axisA = Vector3.zero();
			_rbAFrame.basis.getColumnWith(2, axisA);
			centerOfMassA.basis.transform(axisA);

			Vector3 axisB = Vector3.zero();
			_rbBFrame.basis.getColumnWith(2, axisB);
			centerOfMassB.basis.transform(axisB);

			Vector3 angVelA = getRigidBodyA().getAngularVelocity(Vector3.zero());
			Vector3 angVelB = getRigidBodyB().getAngularVelocity(Vector3.zero());

			Vector3 angVelAroundHingeAxisA = Vector3.zero();
			angVelAroundHingeAxisA.scaleFrom(axisA.dot(angVelA), axisA);

			Vector3 angVelAroundHingeAxisB = Vector3.zero();
			angVelAroundHingeAxisB.scaleFrom(axisB.dot(angVelB), axisB);

			Vector3 angAorthog = Vector3.zero();
			angAorthog.sub2(angVelA, angVelAroundHingeAxisA);

			Vector3 angBorthog = Vector3.zero();
			angBorthog.sub2(angVelB, angVelAroundHingeAxisB);

			Vector3 velrelOrthog = Vector3.zero();
			velrelOrthog.sub2(angAorthog, angBorthog);

			{
				// solve orthogonal angular velocity correction
				double relaxation = 1;
				double len = velrelOrthog.length;
				if (len > 0.00001) {
					Vector3 normal = Vector3.zero();
					normal.normalizeFrom(velrelOrthog);

					double denom = getRigidBodyA().computeAngularImpulseDenominator(normal) +
							getRigidBodyB().computeAngularImpulseDenominator(normal);
					// scale for mass and relaxation
					// todo:  expose this 0.9 factor to developer
					velrelOrthog.scale((1 / denom) * _relaxationFactor);
				}

				// solve angular positional correction
				// TODO: check
				//Vector3 angularError = -axisA.cross(axisB) *(btScalar(1.)/timeStep);
				Vector3 angularError = Vector3.zero();
				angularError.cross2(axisA, axisB);
				angularError.negate();
				angularError.scale(1 / timeStep);
				double len2 = angularError.length;
				if (len2 > 0.00001) {
					Vector3 normal2 = Vector3.zero();
					normal2.normalizeFrom(angularError);

					double denom2 = getRigidBodyA().computeAngularImpulseDenominator(normal2) +
							getRigidBodyB().computeAngularImpulseDenominator(normal2);
					angularError.scale((1 / denom2) * relaxation);
				}

				tmp.negateFrom(velrelOrthog);
				tmp.add(angularError);
				rbA.applyTorqueImpulse(tmp);

				tmp.sub2(velrelOrthog, angularError);
				rbB.applyTorqueImpulse(tmp);

				// solve limit
				if (_solveLimit) {
					tmp.sub2(angVelB, angVelA);
					double amplitude = ((tmp).dot(axisA) * _relaxationFactor + _correction * (1 / timeStep) * _biasFactor) * _limitSign;

					double impulseMag = amplitude * _kHinge;

					// Clamp the accumulated impulse
					double temp = _accLimitImpulse;
					_accLimitImpulse = max(_accLimitImpulse + impulseMag, 0);
					impulseMag = _accLimitImpulse - temp;

					Vector3 impulse = Vector3.zero();
					impulse.scaleFrom(impulseMag * _limitSign, axisA);

					rbA.applyTorqueImpulse(impulse);

					tmp.negateFrom(impulse);
					rbB.applyTorqueImpulse(tmp);
				}
			}

			// apply motor
			if (_enableAngularMotor) {
				// todo: add limits too
				Vector3 angularLimit = Vector3.zero();
				angularLimit.setValues(0, 0, 0);

				Vector3 velrel = Vector3.zero();
				velrel.sub2(angVelAroundHingeAxisA, angVelAroundHingeAxisB);
				double projRelVel = velrel.dot(axisA);

				double desiredMotorVel = _motorTargetVelocity;
				double motorRelvel = desiredMotorVel - projRelVel;

				double unclippedMotorImpulse = _kHinge * motorRelvel;
				// todo: should clip against accumulated impulse
				double clippedMotorImpulse = unclippedMotorImpulse > _maxMotorImpulse ? _maxMotorImpulse : unclippedMotorImpulse;
				clippedMotorImpulse = clippedMotorImpulse < -_maxMotorImpulse ? -_maxMotorImpulse : clippedMotorImpulse;
				Vector3 motorImp = Vector3.zero();
				motorImp.scaleFrom(clippedMotorImpulse, axisA);

				tmp.add2(motorImp, angularLimit);
				rbA.applyTorqueImpulse(tmp);

				tmp.negateFrom(motorImp);
				tmp.sub(angularLimit);
				rbB.applyTorqueImpulse(tmp);
			}
		}
	}

	void updateRHS(double timeStep) {
	}

	double getHingeAngle() {
		Transform centerOfMassA = rbA.getCenterOfMassTransform(Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(Transform());
		
		Vector3 refAxis0 = Vector3.zero();
		_rbAFrame.basis.getColumnWith(0, refAxis0);
		centerOfMassA.basis.transform(refAxis0);

		Vector3 refAxis1 = Vector3.zero();
		_rbAFrame.basis.getColumnWith(1, refAxis1);
		centerOfMassA.basis.transform(refAxis1);

		Vector3 swingAxis = Vector3.zero();
		_rbBFrame.basis.getColumnWith(1, swingAxis);
		centerOfMassB.basis.transform(swingAxis);

		return ScalarUtil.atan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
	}
	
	void setAngularOnly(bool angularOnly) {
		_angularOnly = angularOnly;
	}

	void enableAngularMotor(bool enableMotor, double targetVelocity, double maxMotorImpulse) {
		_enableAngularMotor = enableMotor;
		_motorTargetVelocity = targetVelocity;
		_maxMotorImpulse = maxMotorImpulse;
	}

	void setLimit(double low, double high, [double softness = 0.9, double biasFactor = 0.3, double relaxationFactor = 1.0]) {
		_lowerLimit = low;
		_upperLimit = high;

		_limitSoftness = softness;
		_biasFactor = biasFactor;
		_relaxationFactor = relaxationFactor;
	}

	double getLowerLimit() {
		return _lowerLimit;
	}

	double getUpperLimit() {
		return _upperLimit;
	}

	Transform getAFrame(Transform out) {
		out.copy(_rbAFrame);
		return out;
	}

	Transform getBFrame(Transform out) {
		out.copy(_rbBFrame);
		return out;
	}

	bool getSolveLimit() {
		return _solveLimit;
	}

	double getLimitSign() {
		return _limitSign;
	}

	bool getAngularOnly() {
		return _angularOnly;
	}

	bool getEnableAngularMotor() {
		return _enableAngularMotor;
	}

	double getMotorTargetVelosity() {
		return _motorTargetVelocity;
	}

	double getMaxMotorImpulse() {
		return _maxMotorImpulse;
	}
	
}
