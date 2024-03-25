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

import 'package:bullet_physics/dynamics/constraintsolver/contact_solver_func.dart';
import 'package:vector_math/vector_math.dart';

/**
 * Stores some extra information to each contact point. It is not in the contact
 * point, because that want to keep the collision detection independent from the
 * constraint solver.
 * 
 * @author jezek2
 */
class ConstraintPersistentData {
	
	/** total applied impulse during most recent frame */
	double appliedImpulse = 0;
	double prevAppliedImpulse = 0;
	double accumulatedTangentImpulse0 = 0;
	double accumulatedTangentImpulse1 = 0;

	double jacDiagABInv = 0;
	double jacDiagABInvTangent0 = 0;
	double jacDiagABInvTangent1 = 0;
	int persistentLifeTime = 0;
	double restitution = 0;
	double friction = 0;
	double penetration = 0;
	final Vector3 frictionWorldTangential0 = Vector3.zero();
	final Vector3 frictionWorldTangential1 = Vector3.zero();

	final Vector3 frictionAngularComponent0A = Vector3.zero();
	final Vector3 frictionAngularComponent0B = Vector3.zero();
	final Vector3 frictionAngularComponent1A = Vector3.zero();
	final Vector3 frictionAngularComponent1B = Vector3.zero();

	//some data doesn't need to be persistent over frames: todo: clean/reuse this
	final Vector3 angularComponentA = Vector3.zero();
	final Vector3 angularComponentB = Vector3.zero();

	ContactSolverFunc? contactSolverFunc;
	ContactSolverFunc? frictionSolverFunc;
	
	void reset() {
		appliedImpulse = 0;
		prevAppliedImpulse = 0;
		accumulatedTangentImpulse0 = 0;
		accumulatedTangentImpulse1 = 0;

		jacDiagABInv = 0;
		jacDiagABInvTangent0 = 0;
		jacDiagABInvTangent1 = 0;
		persistentLifeTime = 0;
		restitution = 0;
		friction = 0;
		penetration = 0;
		frictionWorldTangential0.setValues(0, 0, 0);
		frictionWorldTangential1.setValues(0, 0, 0);

		frictionAngularComponent0A.setValues(0, 0, 0);
		frictionAngularComponent0B.setValues(0, 0, 0);
		frictionAngularComponent1A.setValues(0, 0, 0);
		frictionAngularComponent1B.setValues(0, 0, 0);

		angularComponentA.setValues(0, 0, 0);
		angularComponentB.setValues(0, 0, 0);

		contactSolverFunc = null;
		frictionSolverFunc = null;
	}
	
}
