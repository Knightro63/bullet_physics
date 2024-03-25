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

import "package:bullet_physics/dynamics/constraintsolver/jacobian_entry.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint_type.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * Point to point constraint between two rigid bodies each with a pivot point that
 * descibes the "ballsocket" location in local space.
 * 
 * @author jezek2
 */
class Point2PointConstraint extends TypedConstraint {
	final List<JacobianEntry> _jac = [JacobianEntry(), JacobianEntry(), JacobianEntry()]; // 3 orthogonal linear constraints
	final Vector3 _pivotInA = Vector3.zero();
	final Vector3 _pivotInB = Vector3.zero();

	ConstraintSetting setting = ConstraintSetting();

	Point2PointConstraint([RigidBody? rbA, Vector3? pivotInA, RigidBody? rbB, Vector3? pivotInB]):super(TypedConstraintType.point2Point, rbA, rbB) {
    if(pivotInA != null){
		  _pivotInA.setFrom(pivotInA);
    }
    if(pivotInB != null){
		  _pivotInB.setFrom(pivotInB);
    }

    if(rbB == null && rbA != null){
      rbA.getCenterOfMassTransform(Transform()).transform(_pivotInB);
    }
	}

	@override
	void buildJacobian() {
		appliedImpulse = 0;

		Vector3 normal = Vector3.zero();
		normal.setValues(0, 0, 0);

		Matrix3 tmpMat1 = Matrix3.zero();
		Matrix3 tmpMat2 = Matrix3.zero();
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();
		Vector3 tmpVec = Vector3.zero();
		
		Transform centerOfMassA = rbA.getCenterOfMassTransform(Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(Transform());

		for (int i = 0; i < 3; i++) {
			VectorUtil.setCoord(normal, i, 1);

			tmpMat1.transposeFrom(centerOfMassA.basis);
			tmpMat2.transposeFrom(centerOfMassB.basis);

			tmp1.setFrom(_pivotInA);
			centerOfMassA.transform(tmp1);
			tmp1.sub(rbA.getCenterOfMassPosition(tmpVec));

			tmp2.setFrom(_pivotInB);
			centerOfMassB.transform(tmp2);
			tmp2.sub(rbB.getCenterOfMassPosition(tmpVec));

			_jac[i].initAll(
        tmpMat1,
        tmpMat2,
        tmp1,
        tmp2,
        normal,
        rbA.getInvInertiaDiagLocal(Vector3.zero()),
        rbA.getInvMass(),
        rbB.getInvInertiaDiagLocal(Vector3.zero()),
        rbB.getInvMass()
      );
			VectorUtil.setCoord(normal, i, 0);
		}
	}

	@override
	void solveConstraint(double timeStep) {
		Vector3 tmp = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();
		Vector3 tmpVec = Vector3.zero();

		Transform centerOfMassA = rbA.getCenterOfMassTransform(Transform());
		Transform centerOfMassB = rbB.getCenterOfMassTransform(Transform());
		
		Vector3 pivotAInW = Vector3.copy(_pivotInA);
		centerOfMassA.transform(pivotAInW);

		Vector3 pivotBInW = Vector3.copy(_pivotInB);
		centerOfMassB.transform(pivotBInW);

		Vector3 normal = Vector3.zero();
		normal.setValues(0, 0, 0);

		//btVector3 angvelA = m_rbA.getCenterOfMassTransform().getBasis().transpose() * m_rbA.getAngularVelocity();
		//btVector3 angvelB = m_rbB.getCenterOfMassTransform().getBasis().transpose() * m_rbB.getAngularVelocity();

		for (int i = 0; i < 3; i++) {
			VectorUtil.setCoord(normal, i, 1);
			double jacDiagABInv = 1 / _jac[i].getDiagonal();

			Vector3 relPos1 = Vector3.zero();
			relPos1.sub2(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
			Vector3 relPos2 = Vector3.zero();
			relPos2.sub2(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
			// this jacobian entry could be re-used for all iterations

			Vector3 vel1 = rbA.getVelocityInLocalPoint(relPos1, Vector3.zero());
			Vector3 vel2 = rbB.getVelocityInLocalPoint(relPos2, Vector3.zero());
			Vector3 vel = Vector3.zero();
			vel.sub2(vel1, vel2);

			double relVel;
			relVel = normal.dot(vel);

			/*
			//velocity error (first order error)
			btScalar relVel = m_jac[i].getRelativeVelocity(m_rbA.getLinearVelocity(),angvelA,
			m_rbB.getLinearVelocity(),angvelB);
			 */

			// positional error (zeroth order error)
			tmp.sub2(pivotAInW, pivotBInW);
			double depth = -tmp.dot(normal); //this is the error projected on the normal

			double impulse = depth * setting.tau / timeStep * jacDiagABInv - setting.damping * relVel * jacDiagABInv;

			double impulseClamp = setting.impulseClamp;
			if (impulseClamp > 0) {
				if (impulse < -impulseClamp) {
					impulse = -impulseClamp;
				}
				if (impulse > impulseClamp) {
					impulse = impulseClamp;
				}
			}

			appliedImpulse += impulse;
			Vector3 impulseVector = Vector3.zero();
			impulseVector.scaleFrom(impulse, normal);
			tmp.sub2(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));
			rbA.applyImpulse(impulseVector, tmp);
			tmp.negateFrom(impulseVector);
			tmp2.sub2(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));
			rbB.applyImpulse(tmp, tmp2);

			VectorUtil.setCoord(normal, i, 0);
		}
	}
	
	void updateRHS(double timeStep) {
	}

	void setPivotA(Vector3 pivotA) {
		_pivotInA.setFrom(pivotA);
	}

	void setPivotB(Vector3 pivotB) {
		_pivotInB.setFrom(pivotB);
	}

	Vector3 getPivotInA(Vector3 out) {
		out.setFrom(_pivotInA);
		return out;
	}

	Vector3 getPivotInB(Vector3 out) {
		out.setFrom(_pivotInB);
		return out;
	}	
}


////////////////////////////////////////////////////////////////////////////

class ConstraintSetting {
  double tau = 0.3;
  double damping = 1;
  double impulseClamp = 0;
}