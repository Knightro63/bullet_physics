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

import 'package:bullet_physics/dynamics/constraintsolver/solver_constraint_type.dart';
import 'package:vector_math/vector_math.dart';

/**
 * 1D constraint aint a normal axis between bodyA and bodyB. It can be combined
 * to solve contact and friction constraints.
 * 
 * @author jezek2
 */
class SolverConstraint {

	final Vector3 relpos1CrossNormal = Vector3.zero();
	final Vector3 contactNormal = Vector3.zero();

	final Vector3 relpos2CrossNormal = Vector3.zero();
	final Vector3 angularComponentA = Vector3.zero();

	final Vector3 angularComponentB = Vector3.zero();
	
	double appliedPushImpulse = 0;
	
	double appliedImpulse = 0;
	int solverBodyIdA = 0;
	int solverBodyIdB = 0;
	
	double friction = 0;
	double restitution = 0;
	double jacDiagABInv = 0;
	double penetration = 0;
	
	SolverConstraintType constraintType = SolverConstraintType.contact1d;
	int frictionIndex = 0;
	Object? originalContactPoint;
}
