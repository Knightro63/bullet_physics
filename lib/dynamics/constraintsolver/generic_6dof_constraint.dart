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
import "package:bullet_physics/dynamics/constraintsolver/jacobian_entry.dart";
import "package:bullet_physics/dynamics/constraintsolver/rotational_limit_motor.dart";
import "package:bullet_physics/dynamics/constraintsolver/translational_limit_motor.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint_type.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';
/*!

*/
/**
 * Generic6DofConstraint between two rigid bodies each with a pivot point that describes
 * the axis location in local space.<p>
 * 
 * Generic6DofConstraint can leave any of the 6 degree of freedom "free" or "locked".
 * Currently this limit supports rotational motors.<br>
 * 
 * <ul>
 * <li>For linear limits, use {@link #setLinearUpperLimit}, {@link #setLinearLowerLimit}.
 * You can set the parameters with the {@link TranslationalLimitMotor} structure accessible
 * through the {@link #getTranslationalLimitMotor} method. </li>
 * 
 * <li>For angular limits, use the {@link RotationalLimitMotor} structure for configuring
 * the limit. This is accessible through {@link #getRotationalLimitMotor} method,
 * this brings support for limit parameters and motors.</li>
 * 
 * <li>Angulars limits have these possible ranges:
 * <table border="1">
 * <tr>
 * 	<td><b>AXIS</b></td>
 * 	<td><b>MIN ANGLE</b></td>
 * 	<td><b>MAX ANGLE</b></td>
 * </tr><tr>
 * 	<td>X</td>
 * 		<td>-PI</td>
 * 		<td>PI</td>
 * </tr><tr>
 * 	<td>Y</td>
 * 		<td>-PI/2</td>
 * 		<td>PI/2</td>
 * </tr><tr>
 * 	<td>Z</td>
 * 		<td>-PI/2</td>
 * 		<td>PI/2</td>
 * </tr>
 * </table>
 * </li>
 * </ul>
 * 
 * @author jezek2
 */
class Generic6DofConstraint extends TypedConstraint {
  final Transform frameInA = Transform(); //!< the constraint space w.r.t body A
  final Transform frameInB = Transform(); //!< the constraint space w.r.t body B
  final List<JacobianEntry> jacLinear/*[3]*/ = [JacobianEntry(), JacobianEntry(), JacobianEntry()]; //!< 3 orthogonal linear constraints
  final List<JacobianEntry> jacAng/*[3]*/ = [JacobianEntry(), JacobianEntry(), JacobianEntry()]; //!< 3 orthogonal angular constraints
  final TranslationalLimitMotor linearLimits = TranslationalLimitMotor();
  final List<RotationalLimitMotor> angularLimits/*[3]*/ = [RotationalLimitMotor(), RotationalLimitMotor(), RotationalLimitMotor()];
  double timeStep = 0;
  final Transform calculatedTransformA = Transform();
  final Transform calculatedTransformB = Transform();
  final Vector3 calculatedAxisAngleDiff = Vector3.zero();
  final List<Vector3> calculatedAxis/*[3]*/ = [Vector3.zero(), Vector3.zero(), Vector3.zero()];
  final Vector3 anchorPos = Vector3.zero(); // point betwen pivots of bodies A and B to solve linear axes
  final Vector3 calculatedLinearDiff = Vector3.zero();
  bool useLinearReferenceFrameA;

	Generic6DofConstraint([RigidBody? rbB, Transform? frameInB, RigidBody? rbA, Transform? frameInA, this.useLinearReferenceFrameA = true]):super(TypedConstraintType.d6,rbA, rbB) {
		if(frameInA != null){
      this.frameInA.copy(frameInA);
    }
    if(frameInB != null){
		  this.frameInB.copy(frameInB);
    }

    if(rbA == null && frameInA == null){
      rbB?.getCenterOfMassTransform(this.frameInA);
      this.frameInA.mul(frameInB);
    }
	}

	static double _getMatrixElem(Matrix3 mat, int index) {
		int i = index % 3;
		int j = index ~/ 3;
		return mat.getElement(i, j);
	}

	/**
	 * MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html
	 */
	static bool _matrixToEulerXYZ(Matrix3 mat, Vector3 xyz) {
		//	// rot =  cy*cz          -cy*sz           sy
		//	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
		//	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
		//

		if (_getMatrixElem(mat, 2) < 1.0) {
			if (_getMatrixElem(mat, 2) > -1.0) {
				xyz.x = atan2(-_getMatrixElem(mat, 5), _getMatrixElem(mat, 8));
				xyz.y = asin(_getMatrixElem(mat, 2));
				xyz.z = atan2(-_getMatrixElem(mat, 1), _getMatrixElem(mat, 0));
				return true;
			}
			else {
				// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
				xyz.x = -atan2(_getMatrixElem(mat, 3), _getMatrixElem(mat, 4));
				xyz.y = -BulletGlobals.simdHalfPi;
				xyz.z = 0.0;
				return false;
			}
		}
		else {
			// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
			xyz.x = atan2(_getMatrixElem(mat, 3), _getMatrixElem(mat, 4));
			xyz.y = BulletGlobals.simdHalfPi;
			xyz.z = 0.0;
		}

		return false;
	}

	/**
	 * tests linear limits
	 */
  void calculateLinearInfo(){
    calculatedLinearDiff.sub2(calculatedTransformB.origin, calculatedTransformA.origin);

    Matrix3 basisInv = Matrix3.zero();
    basisInv.copyInverse(calculatedTransformA.basis);
    basisInv.transform(calculatedLinearDiff);    // t = this*t      (t is the param)

    linearLimits.currentLinearDiff.setFrom(calculatedLinearDiff);
    for(int i = 0; i < 3; i++){
      linearLimits.testLimitValue(i, VectorUtil.getCoord(calculatedLinearDiff, i) );
    }
  }


	/**
	 * Calcs the euler angles between the two bodies.
	 */
	void calculateAngleInfo() {
		Matrix3 mat = Matrix3.zero();

		Matrix3 relativeFrame = Matrix3.zero();
		mat.setFrom(calculatedTransformA.basis);
		MatrixUtil.invert(mat);
		relativeFrame.mul2(mat, calculatedTransformB.basis);

		_matrixToEulerXYZ(relativeFrame, calculatedAxisAngleDiff);

		// in euler angle mode we do not actually constrain the angular velocity
		// aint the axes axis[0] and axis[2] (although we do use axis[1]) :
		//
		//    to get			constrain w2-w1 aint		...not
		//    ------			---------------------		------
		//    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
		//    d(angle[1])/dt = 0	ax[1]
		//    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
		//
		// constraining w2-w1 aint an axis 'a' means that a'*(w2-w1)=0.
		// to prove the result for angle[0], write the expression for angle[0] from
		// GetInfo1 then take the derivative. to prove this for angle[2] it is
		// easier to take the euler rate expression for d(angle[2])/dt with respect
		// to the components of w and set that to 0.

		Vector3 axis0 = Vector3.zero();
		calculatedTransformB.basis.getColumnWith(0, axis0);

		Vector3 axis2 = Vector3.zero();
		calculatedTransformA.basis.getColumnWith(2, axis2);

		calculatedAxis[1].cross2(axis2, axis0);
		calculatedAxis[0].cross2(calculatedAxis[1], axis2);
		calculatedAxis[2].cross2(axis0, calculatedAxis[1]);

		//    if(m_debugDrawer)
		//    {
		//
		//    	char buff[300];
		//		sprintf(buff,"\n X: %.2 ; Y: %.2 ; Z: %.2 ",
		//		m_calculatedAxisAngleDiff[0],
		//		m_calculatedAxisAngleDiff[1],
		//		m_calculatedAxisAngleDiff[2]);
		//    	m_debugDrawer->reportErrorWarning(buff);
		//    }
	}

	/**
	 * Calcs global transform of the offsets.<p>
	 * Calcs the global transform for the joint offset for body A an B, and also calcs the angle differences between the bodies.
	 * 
	 * See also: Generic6DofConstraint.getCalculatedTransformA, Generic6DofConstraint.getCalculatedTransformB, Generic6DofConstraint.calculateAngleInfo
	 */
	void calculateTransforms() {
		rbA.getCenterOfMassTransform(calculatedTransformA);
		calculatedTransformA.mul(frameInA);

		rbB.getCenterOfMassTransform(calculatedTransformB);
		calculatedTransformB.mul(frameInB);

                calculateLinearInfo();            
		calculateAngleInfo();

	}
	
	void buildLinearJacobian(/*JacobianEntry jacLinear*/int jacLinearIndex, Vector3 normalWorld, Vector3 pivotAInW, Vector3 pivotBInW) {
		Matrix3 mat1 = rbA.getCenterOfMassTransform(Transform()).basis;
		mat1.transpose();

		Matrix3 mat2 = rbB.getCenterOfMassTransform(Transform()).basis;
		mat2.transpose();

		Vector3 tmpVec = Vector3.zero();
		
		Vector3 tmp1 = Vector3.zero();
		tmp1.sub2(pivotAInW, rbA.getCenterOfMassPosition(tmpVec));

		Vector3 tmp2 = Vector3.zero();
		tmp2.sub2(pivotBInW, rbB.getCenterOfMassPosition(tmpVec));

		jacLinear[jacLinearIndex].initAll(
      mat1,
      mat2,
      tmp1,
      tmp2,
      normalWorld,
      rbA.getInvInertiaDiagLocal(Vector3.zero()),
      rbA.getInvMass(),
      rbB.getInvInertiaDiagLocal(Vector3.zero()),
      rbB.getInvMass()
    );
	}

	void buildAngularJacobian(/*JacobianEntry jacAngular*/int jacAngularIndex, Vector3 jointAxisW) {
		Matrix3 mat1 = rbA.getCenterOfMassTransform(Transform()).basis;
		mat1.transpose();

		Matrix3 mat2 = rbB.getCenterOfMassTransform(Transform()).basis;
		mat2.transpose();

		jacAng[jacAngularIndex].initMultiWorld(
      jointAxisW,
      mat1,
      mat2,
      rbA.getInvInertiaDiagLocal(Vector3.zero()),
      rbB.getInvInertiaDiagLocal(Vector3.zero())
    );
	}

	/**
	 * Test angular limit.<p>
	 * Calculates angular correction and returns true if limit needs to be corrected.
	 * Generic6DofConstraint.buildJacobian must be called previously.
	 */
	bool testAngularLimitMotor(int axisIndex) {
		double angle = VectorUtil.getCoord(calculatedAxisAngleDiff, axisIndex);

		// test limits
		angularLimits[axisIndex].testLimitValue(angle);
		return angularLimits[axisIndex].needApplyTorques();
	}
	
	/**
	 * Test linear limit.<p>
	 * Calculates linear correction and returns true if limit needs to be corrected.
	 * Generic6DofConstraint.buildJacobian must be called previously.
	 */
	bool testLinearLimitMotor(int axisIndex) {
		double diff = VectorUtil.getCoord(calculatedLinearDiff, axisIndex);

		// test limits
		linearLimits.testLimitValue(axisIndex, diff); 
		return linearLimits.needApplyForces(axisIndex);
	}

        
        @override
	void buildJacobian() {
		// Clear accumulated impulses for the next simulation step
		linearLimits.accumulatedImpulse.setValues(0, 0, 0);
		for (int i=0; i<3; i++) {
			angularLimits[i].accumulatedImpulse = 0;
		}
		
		// calculates transform
		calculateTransforms();
		
		//Vector3 tmpVec = Vector3.zero();

		//  const btVector3& pivotAInW = m_calculatedTransformA.getOrigin();
		//  const btVector3& pivotBInW = m_calculatedTransformB.getOrigin();
		calcAnchorPos();
		Vector3 pivotAInW = Vector3.copy(anchorPos);
		Vector3 pivotBInW = Vector3.copy(anchorPos);
		
		// not used here
		//    btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition();
		//    btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();

		Vector3 normalWorld = Vector3.zero();
		// linear part
		for (int i=0; i<3; i++) {
			if ( testLinearLimitMotor(i)){
				if (useLinearReferenceFrameA) {
					calculatedTransformA.basis.getColumnWith(i, normalWorld);
				}
				else {
					calculatedTransformB.basis.getColumnWith(i, normalWorld);
				}

				buildLinearJacobian(
						/*jacLinear[i]*/i, normalWorld,
						pivotAInW, pivotBInW);

			}
		}

		// angular part
		for (int i=0; i<3; i++) {
			// calculates error angle
			if (testAngularLimitMotor(i)) {   // test, and true if need to apply force at limit or due to motor
				getAxis(i, normalWorld);
				// Create angular atom
				buildAngularJacobian(/*jacAng[i]*/i, normalWorld);
			}
		}
	}

	@override
	void solveConstraint(double timeStep) {
		this.timeStep = timeStep;

		//calculateTransforms();

		int i;

		// linear

		Vector3 pointInA = Vector3.copy(calculatedTransformA.origin);
		Vector3 pointInB = Vector3.copy(calculatedTransformB.origin);

		double jacDiagABInv;
		Vector3 linearAxis = Vector3.zero();
		for (i = 0; i < 3; i++) {
			if (linearLimits.needApplyForces(i))
                        {
				jacDiagABInv = 1 / jacLinear[i].getDiagonal();

				if (useLinearReferenceFrameA) {
					calculatedTransformA.basis.getColumnWith(i, linearAxis);
				}
				else {
					calculatedTransformB.basis.getColumnWith(i, linearAxis);
				}

				linearLimits.solveLinearAxis(
						this.timeStep,
						jacDiagABInv,
						rbA, pointInA,
						rbB, pointInB,
						i, linearAxis, anchorPos);
			}
		}

		// angular
		Vector3 angularAxis = Vector3.zero();
		double angularJacDiagABInv;
		for (i = 0; i < 3; i++) {
			if (angularLimits[i].needApplyTorques()) { // true if need to apply force at limit or due to motor
				// get axis
				getAxis(i, angularAxis);

				angularJacDiagABInv = 1 / jacAng[i].getDiagonal();

				angularLimits[i].solveAngularLimits(
                                        this.timeStep,
                                        angularAxis,
                                        angularJacDiagABInv,
                                        rbA,
                                        rbB);
			}
		}
	}
	

    void updateRHS(double timeStep) {
	}

	/**
	 * Get the rotation axis in global coordinates.
	 * Generic6DofConstraint.buildJacobian must be called previously.
	 */
	Vector3 getAxis(int axisIndex, Vector3 out) {
		out.setFrom(calculatedAxis[axisIndex]);
		return out;
	}

	/**
	 * Get the relative Euler angle.
	 * Generic6DofConstraint.buildJacobian must be called previously.
	 */
	double getAngle(int axisIndex) {
		return VectorUtil.getCoord(calculatedAxisAngleDiff, axisIndex);
	}

	/**
	 * Gets the global transform of the offset for body A.<p>
	 * See also: Generic6DofConstraint.getFrameOffsetA, Generic6DofConstraint.getFrameOffsetB, Generic6DofConstraint.calculateAngleInfo.
	 */
	Transform getCalculatedTransformA(Transform out) {
		out.copy(calculatedTransformA);
		return out;
	}

	/**
	 * Gets the global transform of the offset for body B.<p>
	 * See also: Generic6DofConstraint.getFrameOffsetA, Generic6DofConstraint.getFrameOffsetB, Generic6DofConstraint.calculateAngleInfo.
	 */
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
	
	void setLinearLowerLimit(Vector3 linearLower) {
		linearLimits.lowerLimit.setFrom(linearLower);
	}

	void setLinearUpperLimit(Vector3 linearUpper) {
		linearLimits.upperLimit.setFrom(linearUpper);
	}

	void setAngularLowerLimit(Vector3 angularLower) {
		angularLimits[0].loLimit = angularLower.x;
		angularLimits[1].loLimit = angularLower.y;
		angularLimits[2].loLimit = angularLower.z;
	}

	void setAngularUpperLimit(Vector3 angularUpper) {
		angularLimits[0].hiLimit = angularUpper.x;
		angularLimits[1].hiLimit = angularUpper.y;
		angularLimits[2].hiLimit = angularUpper.z;
	}

	/**
	 * Retrieves the angular limit information.
	 */
	RotationalLimitMotor getRotationalLimitMotor(int index) {
		return angularLimits[index];
	}

	/**
	 * Retrieves the limit information.
	 */
	TranslationalLimitMotor getTranslationalLimitMotor() {
		return linearLimits;
	}

	/**
	 * first 3 are linear, next 3 are angular
	 */
	void setLimit(int axis, double lo, double hi) {
		if (axis < 3) {
			VectorUtil.setCoord(linearLimits.lowerLimit, axis, lo);
			VectorUtil.setCoord(linearLimits.upperLimit, axis, hi);
		}
		else {
			angularLimits[axis - 3].loLimit = lo;
			angularLimits[axis - 3].hiLimit = hi;
		}
	}
	
	/**
	 * Test limit.<p>
	 * - free means upper &lt; lower,<br>
	 * - locked means upper == lower<br>
	 * - limited means upper &gt; lower<br>
	 * - limitIndex: first 3 are linear, next 3 are angular
	 */
	bool isLimited(int limitIndex) {
		if (limitIndex < 3) {
			return linearLimits.isLimited(limitIndex);
		}
		return angularLimits[limitIndex - 3].isLimited();
	}
	
	// overridable
	void calcAnchorPos() {
		double imA = rbA.getInvMass();
		double imB = rbB.getInvMass();
		double weight;
		if (imB == 0) {
			weight = 1;
		}
		else {
			weight = imA / (imA + imB);
		}
		Vector3 pA = calculatedTransformA.origin;
		Vector3 pB = calculatedTransformB.origin;

		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		tmp1.scaleFrom(weight, pA);
		tmp2.scaleFrom(1 - weight, pB);
		anchorPos.add2(tmp1, tmp2);
	}
	
}
