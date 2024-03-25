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

import "package:bullet_physics/dynamics/action_interface.dart";
import "package:bullet_physics/dynamics/constraintsolver/constraint_solver.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_solver_info.dart";
import "package:bullet_physics/collision/dispatch/collision_world.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/dynamics_world_type.dart";
import "package:bullet_physics/dynamics/internal_tick_callback.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/dynamics/vehicle/raycast_vehicle.dart";
import 'package:vector_math/vector_math.dart';

/**
 * DynamicsWorld is the interface class for several dynamics implementation,
 * basic, discrete, parallel, and continuous etc.
 * 
 * @author jezek2
 */
abstract class DynamicsWorld extends CollisionWorld {
	InternalTickCallback? internalTickCallback;
	Object? worldUserInfo;
	
	final ContactSolverInfo solverInfo = ContactSolverInfo();
	
	DynamicsWorld(super.dispatcher, super.broadphasePairCache, super.collisionConfiguration);

	/**
	 * Proceeds the simulation over 'timeStep', units in preferably in seconds.<p>
	 *
	 * By default, Bullet will subdivide the timestep in constant substeps of each
	 * 'fixedTimeStep'.<p>
	 *
	 * In order to keep the simulation real-time, the maximum number of substeps can
	 * be clamped to 'maxSubSteps'.<p>
	 * 
	 * You can disable subdividing the timestep/substepping by passing maxSubSteps=0
	 * as second argument to stepSimulation, but in that case you have to keep the
	 * timeStep constant.
	 */
	int stepSimulation(double timeStep, [int maxSubSteps = 1, double fixedTimeStep = 1/60]);
	void debugDrawWorld();
	void addConstraint(TypedConstraint constraint, [bool disableCollisionsBetweenLinkedBodies = false]) {}
	void removeConstraint(TypedConstraint constraint) {}
	void addAction(ActionInterface action) {}
	void removeAction(ActionInterface action) {}
	void addVehicle(RaycastVehicle vehicle) {}
	void removeVehicle(RaycastVehicle vehicle) {}

	/**
	 * Once a rigidbody is added to the dynamics world, it will get this gravity assigned.
	 * Existing rigidbodies in the world get gravity assigned too, during this method.
	 */
	void setGravity(Vector3 gravity);
	
	Vector3 getGravity(Vector3 out);
	void addRigidBody(RigidBody body,[int group, int mask]);
	void removeRigidBody(RigidBody body);
	void setConstraintSolver(ConstraintSolver solver);
	ConstraintSolver getConstraintSolver();

	int getNumConstraints() {
		return 0;
	}

	TypedConstraint? getConstraint(int index) {
		return null;
	}

	// JAVA NOTE: not part of the original api
	int getNumActions() {
		return 0;
	}

	// JAVA NOTE: not part of the original api
	ActionInterface? getAction(int index) {
		return null;
	}

	DynamicsWorldType getWorldType();

	void clearForces();
	
	/**
	 * Set the callback for when an internal tick (simulation substep) happens, optional user info.
	 */
	void setInternalTickCallback(InternalTickCallback cb, Object worldUserInfo) {
		internalTickCallback = cb;
		this.worldUserInfo = worldUserInfo;
	}

	void setWorldUserInfo(Object worldUserInfo) {
		this.worldUserInfo = worldUserInfo;
	}

	Object? getWorldUserInfo() {
		return worldUserInfo;
	}

	ContactSolverInfo getSolverInfo() {
		return solverInfo;
	}
	
}
