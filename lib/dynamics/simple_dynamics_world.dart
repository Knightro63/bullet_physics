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

import "package:bullet_physics/collision/broadphase/broadphase_interface.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/collision_dispatcher.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/dynamics/constraintsolver/constraint_solver.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_solver_info.dart";
import "package:bullet_physics/dynamics/dynamics_world.dart";
import "package:bullet_physics/dynamics/dynamics_world_type.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

/**
 * SimpleDynamicsWorld serves as unit-test and to verify more complicated and
 * optimized dynamics worlds. Please use {@link DiscreteDynamicsWorld} instead
 * (or ContinuousDynamicsWorld once it is finished).
 * 
 * @author jezek2
 */
class SimpleDynamicsWorld extends DynamicsWorld {

	ConstraintSolver constraintSolver;
	bool ownsConstraintSolver = false;
	final Vector3 gravity = Vector3(0, 0, -10);
	
	SimpleDynamicsWorld(
    super.dispatcher, 
    super.pairCache, 
    this.constraintSolver, 
    super.collisionConfiguration
  );
  
	void predictUnconstraintMotion(double timeStep) {
		Transform tmpTrans = Transform();
		
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);
			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null) {
				if (!body.isStaticObject()) {
					if (body.isActive()) {
						body.applyGravity();
						body.integrateVelocities(timeStep);
						body.applyDamping(timeStep);
						body.predictIntegratedTransform(timeStep, body.getInterpolationWorldTransform(tmpTrans));
					}
				}
			}
		}
	}
	
	void integrateTransforms(double timeStep) {
		Transform predictedTrans = Transform();
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);
			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null) {
				if (body.isActive() && (!body.isStaticObject())) {
					body.predictIntegratedTransform(timeStep, predictedTrans);
					body.proceedToTransform(predictedTrans);
				}
			}
		}
	}
	
	/**
	 * maxSubSteps/fixedTimeStep for interpolation is currently ignored for SimpleDynamicsWorld, use DiscreteDynamicsWorld instead.
	 */
	@override
	int stepSimulation(double timeStep, [int maxSubSteps = 1, double fixedTimeStep = 1/60]) {
		// apply gravity, predict motion
		predictUnconstraintMotion(timeStep);

		DispatcherInfo dispatchInfo = getDispatchInfo();
		dispatchInfo.timeStep = timeStep;
		dispatchInfo.stepCount = 0;
		dispatchInfo.debugDraw = getDebugDrawer();

		// perform collision detection
		performDiscreteCollisionDetection();

		// solve contact constraints
		int numManifolds = dispatcher1?.getNumManifolds() ?? 0;
		if (numManifolds != 0){
			ObjectArrayList<PersistentManifold>? manifoldPtr = (dispatcher1 as CollisionDispatcher?)?.getInternalManifoldPointer();

			ContactSolverInfo infoGlobal = ContactSolverInfo();
			infoGlobal.timeStep = timeStep;
			constraintSolver.prepareSolve(0,numManifolds);
			constraintSolver.solveGroup(null,0,manifoldPtr, 0, numManifolds, null,0,0,infoGlobal,debugDrawer/*, m_stackAlloc*/,dispatcher1);
			constraintSolver.allSolved(infoGlobal,debugDrawer/*, m_stackAlloc*/);
		}

		// integrate transforms
		integrateTransforms(timeStep);

		updateAabbs();

		synchronizeMotionStates();

		clearForces();

		return 1;
	}

	@override
	void clearForces() {
		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);

			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null) {
				body.clearForces();
			}
		}
	}

	@override
	void setGravity(Vector3 gravity) {
		this.gravity.setFrom(gravity);
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);
			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null) {
				body.setGravity(gravity);
			}
		}
	}

	@override
	Vector3 getGravity(Vector3 out) {
		out.setFrom(gravity);
		return out;
	}

	@override
	void addRigidBody(RigidBody body, [int? group, int? mask]) {
		body.setGravity(gravity);

		if (body.getCollisionShape() != null) {
			addCollisionObject(body);
		}
	}

	@override
	void removeRigidBody(RigidBody body) {
		removeCollisionObject(body);
	}

	@override
	void updateAabbs() {
		Transform tmpTrans = Transform();
		//Transform predictedTrans = Transform();
		Vector3 minAabb = Vector3.zero(), maxAabb = Vector3.zero();

		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);
			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null) {
				if (body.isActive() && (!body.isStaticObject())) {
					colObj?.getCollisionShape()?.getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
					BroadphaseInterface bp = getBroadphase();
					bp.setAabb(body.getBroadphaseHandle(), minAabb, maxAabb, dispatcher1);
				}
			}
		}
	}

	void synchronizeMotionStates() {
		Transform tmpTrans = Transform();
		
		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);
			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null && body.getMotionState() != null) {
				if (body.getActivationState() != CollisionObject.islandSleeping) {
					body.getMotionState()?.setWorldTransform(body.getWorldTransform(tmpTrans));
				}
			}
		}
	}

	@override
	void setConstraintSolver(ConstraintSolver solver) {
		if (ownsConstraintSolver) {
			//btAlignedFree(m_constraintSolver);
		}

		ownsConstraintSolver = false;
		constraintSolver = solver;
	}

	@override
	ConstraintSolver getConstraintSolver() {
		return constraintSolver;
	}
	
	@override
	void debugDrawWorld() {
		// TODO: throw UnsupportedOperationException("Not supported yet.");
	}

	@override
	DynamicsWorldType getWorldType() {
		throw("Not supported yet.");
	}
}
