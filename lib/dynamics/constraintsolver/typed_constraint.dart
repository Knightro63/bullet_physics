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

import 'package:bullet_physics/dynamics/constraintsolver/contact_solver_info.dart';
import 'package:bullet_physics/dynamics/constraintsolver/typed_constraint_type.dart';
import 'package:bullet_physics/dynamics/rigid_body.dart';
import 'package:vector_math/vector_math.dart';

/**
 * TypedConstraint is the base class for Bullet constraints and vehicles.
 * 
 * @author jezek2
 */
abstract class TypedConstraint {
	//final BulletStack stack = BulletStack.get();
	static RigidBody? _sFixed;// = RigidBody(0, null, null);
	
	static RigidBody getFixed(){
		_sFixed ??= RigidBody();
		return _sFixed!;
	}

	int _userConstraintType = -1;
	int _userConstraintId = -1;

	late TypedConstraintType constraintType;
	
	late RigidBody rbA;
	late RigidBody rbB;
	double appliedImpulse = 0;
	
	TypedConstraint(TypedConstraintType type, [RigidBody? rbA, RigidBody? rbB]) {
		constraintType = type;
		this.rbA = rbA ?? getFixed();
		this.rbB = rbB ?? getFixed();
		getFixed().setMassProps(0, Vector3(0, 0, 0));
	}
	
	void buildJacobian();
	void solveConstraint(double timeStep);
	
	RigidBody getRigidBodyA() {
		return rbA;
	}

	RigidBody getRigidBodyB() {
		return rbB;
	}

	int getUserConstraintType() {
		return _userConstraintType;
	}
	
	void setUserConstraintType(int userConstraintType) {
		_userConstraintType = userConstraintType;
	}

	int getUserConstraintId() {
		return _userConstraintId;
	}

	int getUid() {
		return _userConstraintId;
	}

	void setUserConstraintId(int userConstraintId) {
		_userConstraintId = userConstraintId;
	}

	double getAppliedImpulse() {
		return appliedImpulse;
	}

	TypedConstraintType getConstraintType() {
		return constraintType;
	}

  // added to Java port for the Generic6DofSpringConstraint
  // use same name as latest version of Bullet, for consistency, 
  // even though the name doesn't properly reflect function here
  void getInfo2(ContactSolverInfo infoGlobal) {}
}
