/*
 * Dart port of Bullet (c) 2024 @Knightro
 *
 * Bullet Continuous Collision Detection and Physics Library
 * btConeTwistConstraint is Copyright (c) 2007 Starbreeze Studios
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
 * 
 * Written by: Marcus Hennix
 */

import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/dynamics/constraintsolver/jacobian_entry.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint_type.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/quaternion_util.dart";
import "package:bullet_physics/linearmath/scalar_util.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 * ConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc).
 * 
 * @author jezek2
 */
class ConeTwistConstraint extends TypedConstraint {

	final List<JacobianEntry> _jac/*[3]*/ = [ JacobianEntry(), JacobianEntry(), JacobianEntry() ]; //3 orthogonal linear constraints

	final Transform _rbAFrame = Transform();
	final Transform _rbBFrame = Transform();

	double _limitSoftness = 0;
	double _biasFactor = 0.3;
	double _relaxationFactor = 1.0;

	double _swingSpan1 = 1e30;
	double _swingSpan2 = 1e30;
	double _twistSpan = 1e30;

	final Vector3 swingAxis = Vector3.zero();
	final Vector3 twistAxis = Vector3.zero();

	double _kSwing = 0;
	double _kTwist = 0;

	double _twistLimitSign = 0;
	double _swingCorrection = 0;
	double _twistCorrection = 0;

	double _accSwingLimitImpulse = 0;
	double _accTwistLimitImpulse = 0;

	bool _angularOnly = false;
	bool _solveTwistLimit = false;
	bool _solveSwingLimit = false;
	

	ConeTwistConstraint([RigidBody? rbA, RigidBody? rbB, Transform? rbAFrame, Transform? rbBFrame]):super(TypedConstraintType.coneTwist, rbA, rbB) {
		if(rbAFrame != null){
      _rbAFrame.copy(rbAFrame);
    }
    if(rbBFrame != null){
		  _rbBFrame.copy(rbBFrame);
    }
	}
	
	@override
	void buildJacobian() {
		Vector3 tmp = Vector3.zero();
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		Transform tmpTrans = Transform();

		appliedImpulse = 0;

		// set bias, sign, clear accumulator
		_swingCorrection = 0;
		_twistLimitSign = 0;
		_solveTwistLimit = false;
		_solveSwingLimit = false;
		_accTwistLimitImpulse = 0;
		_accSwingLimitImpulse = 0;

		if (!_angularOnly) {
			Vector3 pivotAInW = Vector3.copy(_rbAFrame.origin);
			rbA.getCenterOfMassTransform(tmpTrans).transform(pivotAInW);

			Vector3 pivotBInW = Vector3.copy(_rbBFrame.origin);
			rbB.getCenterOfMassTransform(tmpTrans).transform(pivotBInW);

			Vector3 relPos = Vector3.zero();
			relPos.sub2(pivotBInW, pivotAInW);

			// TODO: stack
			List<Vector3> normal/*[3]*/ = [Vector3.zero(), Vector3.zero(), Vector3.zero()];
			if (relPos.length2 > BulletGlobals.fltEpsilon) {
				normal[0].normalizeFrom(relPos);
			}
			else {
				normal[0].setValues(1, 0, 0);
			}

			TransformUtil.planeSpace1(normal[0], normal[1], normal[2]);

			for (int i = 0; i < 3; i++) {
				Matrix3 mat1 = rbA.getCenterOfMassTransform(Transform()).basis;
				mat1.transpose();

				Matrix3 mat2 = rbB.getCenterOfMassTransform(Transform()).basis;
				mat2.transpose();

				tmp1.sub2(pivotAInW, rbA.getCenterOfMassPosition(tmp));
				tmp2.sub2(pivotBInW, rbB.getCenterOfMassPosition(tmp));

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

		Vector3 b1Axis1 = Vector3.zero(), b1Axis2 = Vector3.zero(), b1Axis3 = Vector3.zero();
		Vector3 b2Axis1 = Vector3.zero(), b2Axis2 = Vector3.zero();

		_rbAFrame.basis.getColumnWith(0, b1Axis1);
		getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis.transform(b1Axis1);

		_rbBFrame.basis.getColumnWith(0, b2Axis1);
		getRigidBodyB().getCenterOfMassTransform(tmpTrans).basis.transform(b2Axis1);

		double swing1 = 0, swing2 = 0;

		double swx = 0, swy = 0;
		double thresh = 10;
		double fact;

		// Get Frame into world space
		if (_swingSpan1 >= 0.05) {
			_rbAFrame.basis.getColumnWith(1, b1Axis2);
			getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis.transform(b1Axis2);
//			swing1 = ScalarUtil.atan2Fast(b2Axis1.dot(b1Axis2), b2Axis1.dot(b1Axis1));
			swx = b2Axis1.dot(b1Axis1);
			swy = b2Axis1.dot(b1Axis2);
			swing1 = ScalarUtil.atan2Fast(swy, swx);
			fact = (swy*swy + swx*swx) * thresh * thresh;
			fact = fact / (fact + 1);
			swing1 *= fact;
		}

		if (_swingSpan2 >= 0.05) {
			_rbAFrame.basis.getColumnWith(2, b1Axis3);
			getRigidBodyA().getCenterOfMassTransform(tmpTrans).basis.transform(b1Axis3);
//			swing2 = ScalarUtil.atan2Fast(b2Axis1.dot(b1Axis3), b2Axis1.dot(b1Axis1));
			swx = b2Axis1.dot(b1Axis1);
			swy = b2Axis1.dot(b1Axis3);
			swing2 = ScalarUtil.atan2Fast(swy, swx);
			fact = (swy*swy + swx*swx) * thresh * thresh;
			fact = fact / (fact + 1);
			swing2 *= fact;
		}

		double rMaxAngle1Sq = 1.0 / (_swingSpan1 * _swingSpan1);
		double rMaxAngle2Sq = 1.0 / (_swingSpan2 * _swingSpan2);
		double ellipseAngle = (swing1*swing1).abs() * rMaxAngle1Sq + (swing2*swing2).abs() * rMaxAngle2Sq;

		if (ellipseAngle > 1.0) {
			_swingCorrection = ellipseAngle - 1.0;
			_solveSwingLimit = true;

			// Calculate necessary axis & factors
			tmp1.scaleFrom(b2Axis1.dot(b1Axis2), b1Axis2);
			tmp2.scaleFrom(b2Axis1.dot(b1Axis3), b1Axis3);
			tmp.add2(tmp1, tmp2);
			swingAxis.cross2(b2Axis1, tmp);
			swingAxis.normalize();

			double swingAxisSign = (b2Axis1.dot(b1Axis1) >= 0.0) ? 1.0 : -1.0;
			swingAxis.scale(swingAxisSign);

			_kSwing = 1 / (getRigidBodyA().computeAngularImpulseDenominator(swingAxis) +
					getRigidBodyB().computeAngularImpulseDenominator(swingAxis));

		}

		// Twist limits
		if (_twistSpan >= 0) {
			//Vector3 b2Axis2 = Vector3.zero();
			_rbBFrame.basis.getColumnWith(1, b2Axis2);
			getRigidBodyB().getCenterOfMassTransform(tmpTrans).basis.transform(b2Axis2);

			Quaternion rotationArc = QuaternionUtil.intestArcQuat(b2Axis1, b1Axis1, Quaternion(0,0,0,0)); 
			Vector3 twistRef = QuaternionUtil.quatRotate(rotationArc, b2Axis2, Vector3.zero());
			double twist = ScalarUtil.atan2Fast(twistRef.dot(b1Axis3), twistRef.dot(b1Axis2));

			double lockedFreeFactor = (_twistSpan > 0.05) ? _limitSoftness : 0;
			if (twist <= -_twistSpan * lockedFreeFactor) {
				_twistCorrection = -(twist + _twistSpan);
				_solveTwistLimit = true;

				twistAxis.add2(b2Axis1, b1Axis1);
				twistAxis.scale(0.5);
				twistAxis.normalize();
				twistAxis.scale(-1.0);

				_kTwist = 1 / (getRigidBodyA().computeAngularImpulseDenominator(twistAxis) +
						getRigidBodyB().computeAngularImpulseDenominator(twistAxis));

			}
			else if (twist > _twistSpan * lockedFreeFactor) {
				_twistCorrection = (twist - _twistSpan);
				_solveTwistLimit = true;

				twistAxis.add2(b2Axis1, b1Axis1);
				twistAxis.scale(0.5);
				twistAxis.normalize();

				_kTwist = 1 / (getRigidBodyA().computeAngularImpulseDenominator(twistAxis) +
						getRigidBodyB().computeAngularImpulseDenominator(twistAxis));
			}
		}
	}

	@override
	void solveConstraint(double timeStep) {
		Vector3 tmp = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		Vector3 tmpVec = Vector3.zero();
		Transform tmpTrans = Transform();

		Vector3 pivotAInW = Vector3.copy(_rbAFrame.origin);
		rbA.getCenterOfMassTransform(tmpTrans).transform(pivotAInW);

		Vector3 pivotBInW = Vector3.copy(_rbBFrame.origin);
		rbB.getCenterOfMassTransform(tmpTrans).transform(pivotBInW);

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
			Vector3 angVelA = getRigidBodyA().getAngularVelocity(Vector3.zero());
			Vector3 angVelB = getRigidBodyB().getAngularVelocity(Vector3.zero());

			// solve swing limit
			if (_solveSwingLimit) {
				tmp.sub2(angVelB, angVelA);
				double amplitude = ((tmp).dot(swingAxis) * _relaxationFactor * _relaxationFactor + _swingCorrection * (1 / timeStep) * _biasFactor);
				double impulseMag = amplitude * _kSwing;

				// Clamp the accumulated impulse
				double temp = _accSwingLimitImpulse;
				_accSwingLimitImpulse = max(_accSwingLimitImpulse + impulseMag, 0.0);
				impulseMag = _accSwingLimitImpulse - temp;

				Vector3 impulse = Vector3.zero();
				impulse.scaleFrom(impulseMag, swingAxis);

				rbA.applyTorqueImpulse(impulse);

				tmp.negateFrom(impulse);
				rbB.applyTorqueImpulse(tmp);
			}

			// solve twist limit
			if (_solveTwistLimit) {
				tmp.sub2(angVelB, angVelA);
				double amplitude = ((tmp).dot(twistAxis) * _relaxationFactor * _relaxationFactor + _twistCorrection * (1 / timeStep) * _biasFactor);
				double impulseMag = amplitude * _kTwist;

				// Clamp the accumulated impulse
				double temp = _accTwistLimitImpulse;
				_accTwistLimitImpulse = max(_accTwistLimitImpulse + impulseMag, 0.0);
				impulseMag = _accTwistLimitImpulse - temp;

				Vector3 impulse = Vector3.zero();
				impulse.scaleFrom(impulseMag, twistAxis);

				rbA.applyTorqueImpulse(impulse);

				tmp.negateFrom(impulse);
				rbB.applyTorqueImpulse(tmp);
			}
		}
	}

	void updateRHS(double timeStep) {
	}

	void setAngularOnly(bool angularOnly) {
		_angularOnly = angularOnly;
	}

	void setLimit(double swingSpan1, double swingSpan2, double twistSpan, [double softness = 0.8, double biasFactor = 0.3, double relaxationFactor = 1.0]) {
		_swingSpan1 = swingSpan1;
		_swingSpan2 = swingSpan2;
		_twistSpan = twistSpan;

		_limitSoftness = softness;
		_biasFactor = biasFactor;
		_relaxationFactor = relaxationFactor;
	}

	Transform getAFrame(Transform out) {
		out.copy(_rbAFrame);
		return out;
	}

	Transform getBFrame(Transform out) {
		out.copy(_rbBFrame);
		return out;
	}

	bool getSolveTwistLimit() {
		return _solveTwistLimit;
	}

	bool getSolveSwingLimit() {
		return _solveTwistLimit;
	}

	double getTwistLimitSign() {
		return _twistLimitSign;
	}
	
}
