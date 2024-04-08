/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

import "package:bullet_physics/dynamics/constraintsolver/contact_solver_info.dart";
import "package:bullet_physics/dynamics/constraintsolver/generic_6dof_constraint.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint_type.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 *
 * Generic 6 DOF constraint that allows to set spring motors to any translational and rotational DOF
 * DOF index used in enableSpring() and setStiffness() means:
 *    0 : translation X
 *    1 : translation Y
 *    2 : translation Z
 *    3 : rotation X (3rd Euler rotational around position of X axis, range [-PI+epsilon, PI-epsilon] )
 *    4 : rotation Y (2nd Euler rotational around position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
 *    5 : rotation Z (1st Euler rotational around Z axis, range [-PI+epsilon, PI-epsilon] )
 *
 * @author Ported to JBullet from Bullet by gideonk as part of the QIntBio project
 */
class Generic6DofSpringConstraint extends Generic6DofConstraint {
  List<bool> _springEnabled= [false,false,false,false,false,false];//bool[6];
  List<double> _equilibriumPoint = [0,0,0,0,0,0];//double[6];
  List<double> _springStiffness = [0,0,0,0,0,0];//double[6];
  List<double> _springDamping = [1,1,1,1,1,1];//double[6]; // between 0 and 1 (1 == no damping)

  Generic6DofSpringConstraint(super.rbA, super.rbB, super.frameInA, super.frameInB, super.useLinearReferenceFrameA){
    constraintType = TypedConstraintType.d6Spring;
  }

  void enableSpring(int index, bool onOff) {
    assert((index >= 0) && (index < 6));
    _springEnabled[index] = onOff;
    if (index < 3) {
        linearLimits.enableMotor[index] = onOff;
    } else {
        angularLimits[index - 3].enableMotor = onOff;
    }
  }

  void setStiffness(int index, double stiffness) {
    assert ((index >= 0) && (index < 6));
    _springStiffness[index] = stiffness;
  }

  void setDamping(int index, double damping) {
    assert ((index >= 0) && (index < 6));
    _springDamping[index] = damping;
  }

  void setEquilibriumPoints() {
    calculateTransforms();
    int i;
    for (i = 0; i < 3; i++) {
      _equilibriumPoint[i] = VectorUtil.getCoord(calculatedLinearDiff, i);
    }
    for (i = 0; i < 3; i++) {
      _equilibriumPoint[i + 3] = VectorUtil.getCoord(calculatedAxisAngleDiff, i);
    }
  }

  void setEquilibriumPoint(int index) {
    assert ((index >= 0) && (index < 6));
    calculateTransforms();
    if (index < 3) {
      _equilibriumPoint[index] = VectorUtil.getCoord(calculatedLinearDiff, index);
    } 
    else {
      _equilibriumPoint[index] = VectorUtil.getCoord(calculatedAxisAngleDiff, index-3);
    }
  }

  void setEquilibriumPointWithValue(int index, double val) {
    assert ((index >= 0) && (index < 6));
    _equilibriumPoint[index] = val;
  }

  void internalUpdateSprings(ContactSolverInfo info) {
    // it is assumed that calculateTransforms() have been called before this call
    int i;

    Vector3 velA = rbA.getLinearVelocity( Vector3.zero() );
    Vector3 velB = rbB.getLinearVelocity( Vector3.zero() );
        
	Vector3 relVel = Vector3.zero();
  relVel.sub2(velB, velA);
  double fps = 1 / info.timeStep;

	for(i = 0; i < 3; i++) {
    if (_springEnabled[i]) {
      // get current position of constraint
      double currPos = VectorUtil.getCoord(calculatedLinearDiff, i);
      // calculate difference
      double delta = currPos - _equilibriumPoint[i];
      // spring force is (delta * stiffness) according to Hooke's Law
      double force = delta * _springStiffness[i];

      double velFactor = fps * _springDamping[i] / info.numIterations;
      VectorUtil.setCoord(linearLimits.targetVelocity, i, velFactor * force);
      VectorUtil.setCoord(linearLimits.maxMotorForce, i, force.abs() / fps);
    }
  }
  for (i = 0; i < 3; i++) {
    if (_springEnabled[i + 3]) {
        // get current position of constraint
        double currPos = VectorUtil.getCoord(calculatedAxisAngleDiff, i);
        // calculate difference
        double delta = currPos - _equilibriumPoint[i + 3];
        // spring force is (-delta * stiffness) according to Hooke's Law
        double force = -delta * _springStiffness[i + 3];
        double velFactor = fps * _springDamping[i + 3] / info.numIterations;
        angularLimits[i].targetVelocity = velFactor * force;
        angularLimits[i].maxMotorForce = force.abs() / fps;
      }
    }
  }

  void setAxis(Vector3 axis1, Vector3 axis2){
    Vector3 zAxis = Vector3.copy(axis1);
    zAxis.normalize();

    Vector3 yAxis = Vector3.copy(axis2);
    zAxis.normalize();

    Vector3 xAxis = Vector3.zero();
    xAxis.cross2(yAxis, zAxis);      // we want right coordinate system

    Transform frameInW = Transform();
    frameInW.setIdentity();

    frameInW.basis.setColumn(0, xAxis);
    frameInW.basis.setColumn(1, yAxis);
    frameInW.basis.setColumn(2, zAxis);


    // now get constraint frame in local coordinate systems
    Transform temp = Transform();

    rbA.getCenterOfMassTransform(temp);
    temp.inverse();
    frameInA.mul2(temp, frameInW);

    rbB.getCenterOfMassTransform(temp);
    temp.inverse();
    frameInB.mul2(temp, frameInW);

    //        // now get constraint frame in local coordinate systems
    //        frameInA = rbA.getCenterOfMassTransform().inverse() * frameInW;
    //        frameInB = rbB.getCenterOfMassTransform().inverse() * frameInW;

    calculateTransforms();
  }


  @override
  void getInfo2(ContactSolverInfo infoGlobal) {
    internalUpdateSprings(infoGlobal);
  }
}
