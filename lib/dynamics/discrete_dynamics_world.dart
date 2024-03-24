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

import "package:bullet_physics/collision/broadphase/collision_filter_groups.dart";
import "package:bullet_physics/collision/dispatch/simulation_island_manager.dart";
import "package:bullet_physics/dynamics/action_interface.dart";
import "package:bullet_physics/dynamics/constraintsolver/constraint_solver.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_solver_info.dart";
import "package:bullet_physics/dynamics/constraintsolver/sequential_impulse_constraint_solver.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/dynamics_world.dart";
import "package:bullet_physics/dynamics/dynamics_world_type.dart";
import "package:bullet_physics/dynamics/internal_tick_callback.dart";
import "package:bullet_physics/dynamics/rigid_body.dart";
import "package:bullet_physics/dynamics/vehicle/raycast_vehicle.dart";
import "package:bullet_physics/linearmath/c_profile_manager.dart";
import "package:bullet_physics/linearmath/debug_draw_modes.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/core/bullet_stats.dart";
import "package:bullet_physics/collision/broadphase/broadphase_pair.dart";
import "package:bullet_physics/collision/broadphase/broadphase_proxy.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/broadphase/overlapping_pair_cache.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/collision_world.dart";
import "package:bullet_physics/collision/narrowphase/manifold_point.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/sphere_shape.dart";
import "package:bullet_physics/linearmath/i_debug_draw.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/linearmath/scalar_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

/**
 * DiscreteDynamicsWorld provides discrete rigid body simulation.
 * 
 * @author jezek2
 */
class DiscreteDynamicsWorld extends DynamicsWorld {

	late ConstraintSolver constraintSolver;
	late SimulationIslandManager islandManager;
	final ObjectArrayList<TypedConstraint> constraints = ObjectArrayList();
	final Vector3 gravity = Vector3(0, -10, 0);

	//for variable timesteps
	double localTime = 1 / 60;
	//for variable timesteps

	bool ownsIslandManager = true;
	bool ownsConstraintSolver = false;

	ObjectArrayList<RaycastVehicle> vehicles = ObjectArrayList();
	
	ObjectArrayList<ActionInterface> actions = ObjectArrayList();

	int profileTimings = 0;
  InternalTickCallback? preTickCallback;
	
	DiscreteDynamicsWorld(super.dispatcher, super.pairCache, ConstraintSolver? constraintSolver, super.collisionConfiguration) {
		this.constraintSolver = constraintSolver ?? SequentialImpulseConstraintSolver();
    if (constraintSolver == null) {
			ownsConstraintSolver = true;
		}
		else {
			ownsConstraintSolver = false;
		}
		islandManager = SimulationIslandManager();
	}

	void saveKinematicState(double timeStep) {
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);
			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null) {
				//Transform predictedTrans = Transform();
				if (body.getActivationState() != CollisionObject.islandSleeping) {
					if (body.isKinematicObject()) {
						// to calculate velocities next frame
						body.saveKinematicState(timeStep);
					}
				}
			}
		}
	}

	@override
	void debugDrawWorld() {
		if (getDebugDrawer() != null && ((getDebugDrawer()?.getDebugMode() ?? 0) & DebugDrawModes.drawContactPoints) != 0) {
			int numManifolds = getDispatcher()?.getNumManifolds() ?? 0;
			Vector3 color = Vector3.zero();
			color.setValues(0, 0, 0);
			for (int i = 0; i < numManifolds; i++) {
				PersistentManifold? contactManifold = getDispatcher()?.getManifoldByIndexInternal(i);
				//btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
				//btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

				int numContacts = contactManifold?.getNumContacts() ?? 0;
				for (int j = 0; j < numContacts; j++) {
					ManifoldPoint cp = contactManifold!.getContactPoint(j);
					getDebugDrawer()?.drawContactPoint(cp.positionWorldOnB, cp.normalWorldOnB, cp.getDistance(), cp.getLifeTime(), color);
				}
			}
		}
		
		if (getDebugDrawer() != null && ((getDebugDrawer()?.getDebugMode() ?? 0) & (DebugDrawModes.drawWireframe | DebugDrawModes.drawAABB)) != 0) {
			int i;

			Transform tmpTrans = Transform();
			Vector3 minAabb = Vector3.zero();
			Vector3 maxAabb = Vector3.zero();
			Vector3 colorvec = Vector3.zero();
			
			// todo: iterate over awake simulation islands!
			for (i = 0; i < collisionObjects.size; i++) {
				CollisionObject? colObj = collisionObjects.getQuick(i);
				if (getDebugDrawer() != null && ((getDebugDrawer()?.getDebugMode() ?? 0) & DebugDrawModes.drawWireframe) != 0) {
					Vector3 color = Vector3.zero();
					color.setValues(255, 255, 255);
					switch (colObj?.getActivationState() ?? 0) {
						case CollisionObject.activeTag:
							color.setValues(255, 255, 255);
							break;
						case CollisionObject.islandSleeping:
							color.setValues(0, 255, 0);
							break;
						case CollisionObject.wantsDeactivation:
							color.setValues(0, 255, 255);
							break;
						case CollisionObject.disableDeactivation:
							color.setValues(255, 0, 0);
							break;
						case CollisionObject.disableSimulation:
							color.setValues(255, 255, 0);
							break;
						default: {
							color.setValues(255, 0, 0);
						}
					}

					debugDrawObject(colObj?.getWorldTransform(tmpTrans), colObj?.getCollisionShape(), color);
				}
				if (debugDrawer != null && ((debugDrawer?.getDebugMode() ?? 0) & DebugDrawModes.drawAABB) != 0) {
					colorvec.setValues(1, 0, 0);
					colObj?.getCollisionShape()?.getAabb(colObj.getWorldTransform(tmpTrans), minAabb, maxAabb);
					debugDrawer?.drawAabb(minAabb, maxAabb, colorvec);
				}
			}

			Vector3 wheelColor = Vector3.zero();
			Vector3 wheelPosWS = Vector3.zero();
			Vector3 axle = Vector3.zero();
			Vector3 tmp = Vector3.zero();

			for (i = 0; i < vehicles.size; i++) {
				for (int v = 0; v < (vehicles.getQuick(i)?.getNumWheels() ?? 0); v++) {
					wheelColor.setValues(0, 255, 255);
					if (vehicles.getQuick(i)?.getWheelInfo(v).raycastInfo.isInContact ?? false) {
						wheelColor.setValues(0, 0, 255);
					}
					else {
						wheelColor.setValues(255, 0, 255);
					}

					wheelPosWS.setFrom(vehicles.getQuick(i)?.getWheelInfo(v).worldTransform.origin ?? Vector3.zero());

					axle.setValues(
							vehicles.getQuick(i)!.getWheelInfo(v).worldTransform.basis.getElement(0, vehicles.getQuick(i)!.getRightAxis()),
							vehicles.getQuick(i)!.getWheelInfo(v).worldTransform.basis.getElement(1, vehicles.getQuick(i)!.getRightAxis()),
							vehicles.getQuick(i)!.getWheelInfo(v).worldTransform.basis.getElement(2, vehicles.getQuick(i)!.getRightAxis()));


					//m_vehicles[i]->getWheelInfo(v).m_raycastInfo.m_wheelAxleWS
					//debug wheels (cylinders)
					tmp.add2(wheelPosWS, axle);
					debugDrawer?.drawLine(wheelPosWS, tmp, wheelColor);
					debugDrawer?.drawLine(wheelPosWS, vehicles.getQuick(i)!.getWheelInfo(v).raycastInfo.contactPointWS, wheelColor);
				}
			}

			if (getDebugDrawer() != null && getDebugDrawer()?.getDebugMode() != 0) {
				for (i=0; i<actions.size; i++) {
					actions.getQuick(i)!.debugDraw(debugDrawer);
				}
			}
		}
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
	
	/**
	 * Apply gravity, call this once per timestep.
	 */
	void applyGravity() {
		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);

			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null && body.isActive()) {
				body.applyGravity();
			}
		}
	}

	void synchronizeMotionStates() {
		Transform interpolatedTransform = Transform();
		
		Transform tmpTrans = Transform();
		Vector3 tmpLinVel = Vector3.zero();
		Vector3 tmpAngVel = Vector3.zero();

		// todo: iterate over awake simulation islands!
		for (int i = 0; i < collisionObjects.size; i++) {
			CollisionObject? colObj = collisionObjects.getQuick(i);

			RigidBody? body = RigidBody.upcast(colObj);
			if (body != null && body.getMotionState() != null && !body.isStaticOrKinematicObject()) {
				// we need to call the update at least once, even for sleeping objects
				// otherwise the 'graphics' transform never updates properly
				// so todo: add 'dirty' flag
				//if (body->getActivationState() != ISLAND_SLEEPING)
				{
					TransformUtil.integrateTransform(
							body.getInterpolationWorldTransform(tmpTrans),
							body.getInterpolationLinearVelocity(tmpLinVel),
							body.getInterpolationAngularVelocity(tmpAngVel),
							localTime * body.getHitFraction(), interpolatedTransform);
					body.getMotionState()?.setWorldTransform(interpolatedTransform);
				}
			}
		}

		if (getDebugDrawer() != null && ((getDebugDrawer()?.getDebugMode() ?? 0) & DebugDrawModes.drawWireframe) != 0) {
			for (int i = 0; i < vehicles.size; i++) {
				for (int v = 0; v < (vehicles.getQuick(i)?.getNumWheels() ?? 0); v++) {
					// synchronize the wheels with the (interpolated) chassis worldtransform
					vehicles.getQuick(i)?.updateWheelTransform(v, true);
				}
			}
		}
	}

	@override
	int stepSimulation(double timeStep, [int maxSubSteps = 1, double fixedTimeStep = 1/60]) {
		startProfiling(timeStep);

		int t0 = DateTime.now().microsecondsSinceEpoch;//System.nanoTime();
		
		BulletStats.pushProfile("stepSimulation");
		try {
			int numSimulationSubSteps = 0;

			if (maxSubSteps != 0) {
				// fixed timestep with interpolation
				localTime += timeStep;
				if (localTime >= fixedTimeStep) {
					numSimulationSubSteps = (localTime ~/ fixedTimeStep);
					localTime -= numSimulationSubSteps * fixedTimeStep;
				}
			}
			else {
				//variable timestep
				fixedTimeStep = timeStep;
				localTime = timeStep;
				if (ScalarUtil.fuzzyZero(timeStep)) {
					numSimulationSubSteps = 0;
					maxSubSteps = 0;
				}
				else {
					numSimulationSubSteps = 1;
					maxSubSteps = 1;
				}
			}
			// process some debugging flags
			if (getDebugDrawer() != null) {
				BulletGlobals.setDeactivationDisabled(((getDebugDrawer()?.getDebugMode() ?? 0) & DebugDrawModes.noDeactivation) != 0);
			}
			if (numSimulationSubSteps != 0) {
				saveKinematicState(fixedTimeStep);
				applyGravity();
				// clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
				int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps) ? maxSubSteps : numSimulationSubSteps;

				for (int i = 0; i < clampedSimulationSteps; i++) {
					internalSingleStepSimulation(fixedTimeStep);
					synchronizeMotionStates();
				}
			}

			synchronizeMotionStates();
			clearForces();

			//#ifndef BT_NO_PROFILE
			CProfileManager.incrementFrameCounter();
			//#endif //BT_NO_PROFILE

			return numSimulationSubSteps;
		}
		finally {
			BulletStats.popProfile();
			BulletStats.stepSimulationTime = (DateTime.now().microsecondsSinceEpoch - t0) ~/ 1000;//(System.nanoTime()- t0) ~/ 1000000
		}
	}

	void internalSingleStepSimulation(double timeStep) {
		BulletStats.pushProfile("internalSingleStepSimulation");
		try {
      if (preTickCallback != null) {
				preTickCallback!.internalTick(this, timeStep);
			}

			// apply gravity, predict motion
			predictUnconstraintMotion(timeStep);

			DispatcherInfo dispatchInfo = getDispatchInfo();

			dispatchInfo.timeStep = timeStep;
			dispatchInfo.stepCount = 0;
			dispatchInfo.debugDraw = getDebugDrawer();

			// perform collision detection
			performDiscreteCollisionDetection();
      
			calculateSimulationIslands();

			getSolverInfo().timeStep = timeStep;

			// solve contact and other joint constraints
			solveConstraints(getSolverInfo());

			//CallbackTriggers();

			// integrate transforms
			integrateTransforms(timeStep);

			// update vehicle simulation
			updateActions(timeStep);

			// update vehicle simulation
			updateVehicles(timeStep);

			updateActivationState(timeStep);

      if (internalTickCallback != null) {
				internalTickCallback!.internalTick(this, timeStep);
			}
		}
		finally {
			BulletStats.popProfile();
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
	void removeRigidBody(RigidBody body) {
		removeCollisionObject(body);
	}

  @override
	void addRigidBody(RigidBody body, [int? group, int? mask]) {
		if (!body.isStaticOrKinematicObject()) {
			body.setGravity(gravity);
		}

    if (body.getCollisionShape() != null) {
      bool isDynamic = !(body.isStaticObject() || body.isKinematicObject());
			int collisionFilterGroup = group ?? (isDynamic ? CollisionFilterGroups.defaultFilter : CollisionFilterGroups.staticFilter);
			int collisionFilterMask = mask ?? (isDynamic ? CollisionFilterGroups.allFilter : (CollisionFilterGroups.allFilter ^ CollisionFilterGroups.staticFilter));
      addCollisionObject(body, collisionFilterGroup, collisionFilterMask);
    }

	}

	void updateActions(double timeStep) {
		BulletStats.pushProfile("updateActions");
		try {
			for (int i=0; i<actions.size; i++) {
				actions.getQuick(i)?.updateAction(this, timeStep);
			}
		}
		finally {
			BulletStats.popProfile();
		}
	}

	void updateVehicles(double timeStep) {
		BulletStats.pushProfile("updateVehicles");
		try {
			for (int i = 0; i < vehicles.size; i++) {
				RaycastVehicle? vehicle = vehicles.getQuick(i);
				vehicle?.updateVehicle(timeStep);
			}
		}
		finally {
			BulletStats.popProfile();
		}
	}

	void updateActivationState(double timeStep) {
		BulletStats.pushProfile("updateActivationState");
		try {
			Vector3 tmp = Vector3.zero();

			for (int i=0; i<collisionObjects.size; i++) {
				CollisionObject? colObj = collisionObjects.getQuick(i);
				RigidBody? body = RigidBody.upcast(colObj);
				if (body != null) {
					body.updateDeactivation(timeStep);

					if (body.wantsSleeping()) {
						if (body.isStaticOrKinematicObject()) {
							body.setActivationState(CollisionObject.islandSleeping);
						}
						else {
							if (body.getActivationState() == CollisionObject.activeTag) {
								body.setActivationState(CollisionObject.wantsDeactivation);
							}
							if (body.getActivationState() == CollisionObject.islandSleeping) {
								tmp.setValues(0, 0, 0);
								body.setAngularVelocity(tmp);
								body.setLinearVelocity(tmp);
							}
						}
					}
					else {
						if (body.getActivationState() != CollisionObject.disableDeactivation) {
							body.setActivationState(CollisionObject.activeTag);
						}
					}
				}
			}
		}
		finally {
			BulletStats.popProfile();
		}
	}

	@override
	void addConstraint(TypedConstraint constraint, [bool disableCollisionsBetweenLinkedBodies = false]) {
		constraints.add(constraint);
		if (disableCollisionsBetweenLinkedBodies) {
			constraint.getRigidBodyA().addConstraintRef(constraint);
			constraint.getRigidBodyB().addConstraintRef(constraint);
		}
	}

	@override
	void removeConstraint(TypedConstraint constraint) {
		constraints.remove(constraint);
		constraint.getRigidBodyA().removeConstraintRef(constraint);
		constraint.getRigidBodyB().removeConstraintRef(constraint);
	}

	@override
	void addAction(ActionInterface action) {
		actions.add(action);
	}

	@override
	void removeAction(ActionInterface action) {
		actions.remove(action);
	}

	@override
	void addVehicle(RaycastVehicle vehicle) {
		vehicles.add(vehicle);
	}
	
	@override
	void removeVehicle(RaycastVehicle vehicle) {
		vehicles.remove(vehicle);
	}
	
	static int _getConstraintIslandId(TypedConstraint? lhs) {
    if(lhs == null) return 0;
		int islandId;

		CollisionObject rcolObj0 = lhs.getRigidBodyA();
		CollisionObject rcolObj1 = lhs.getRigidBodyB();
		islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
		return islandId;
	}

	ObjectArrayList<TypedConstraint> _sortedConstraints = ObjectArrayList();//List<TypedConstraint>();
	_InplaceSolverIslandCallback _solverCallback = _InplaceSolverIslandCallback();
	
	void solveConstraints(ContactSolverInfo solverInfo) {
		BulletStats.pushProfile("solveConstraints");
		try {
			// sorted version of all btTypedConstraint, based on islandId
			_sortedConstraints.clear();
			for (int i=0; i<constraints.size; i++) {
				_sortedConstraints.add(constraints.getQuick(i));
			}
			//Collections.sort(sortedConstraints, sortConstraintOnIslandPredicate);
			MiscUtil.quickSortObjectArray(_sortedConstraints, _sortConstraintOnIslandPredicate);

			ObjectArrayList<TypedConstraint>? constraintsPtr = getNumConstraints() != 0 ? _sortedConstraints : null;

			_solverCallback.init(solverInfo, constraintSolver, constraintsPtr, _sortedConstraints.size, debugDrawer/*,m_stackAlloc*/, dispatcher1);

			constraintSolver.prepareSolve(getCollisionWorld().getNumCollisionObjects(), getCollisionWorld().getDispatcher()?.getNumManifolds() ?? 0);

			// solve all the constraints for this island
			islandManager.buildAndProcessIslands(getCollisionWorld().getDispatcher(), getCollisionWorld().getCollisionObjectArray(), _solverCallback);

			constraintSolver.allSolved(solverInfo, debugDrawer/*, m_stackAlloc*/);
		}
		finally {
			BulletStats.popProfile();
		}
	}

	void calculateSimulationIslands() {
		BulletStats.pushProfile("calculateSimulationIslands");
		try {
			getSimulationIslandManager().updateActivationState(getCollisionWorld(), getCollisionWorld().getDispatcher());

			{
				int i;
				int numConstraints = constraints.size;
				for (i = 0; i < numConstraints; i++) {
					TypedConstraint? constraint = constraints.getQuick(i);

					RigidBody? colObj0 = constraint?.getRigidBodyA();
					RigidBody? colObj1 = constraint?.getRigidBodyB();

					if (((colObj0 != null) && (!colObj0.isStaticOrKinematicObject())) &&
						((colObj1 != null) && (!colObj1.isStaticOrKinematicObject()))){
						if (colObj0.isActive() || colObj1.isActive()) {
							getSimulationIslandManager().getUnionFind().unite((colObj0).getIslandTag(), (colObj1).getIslandTag());
						}
					}
				}
			}

			// Store the island id in each body
			getSimulationIslandManager().storeIslandActivationState(getCollisionWorld());
		}
		finally {
			BulletStats.popProfile();
		}
	}

	void integrateTransforms(double timeStep) {
		BulletStats.pushProfile("integrateTransforms");
		try {
			Vector3 tmp = Vector3.zero();
			Transform tmpTrans = Transform();

			Transform predictedTrans = Transform();
			for (int i=0; i<collisionObjects.size; i++) {
				CollisionObject? colObj = collisionObjects.getQuick(i);
				RigidBody? body = RigidBody.upcast(colObj);
				if (body != null) {
					body.setHitFraction(1);

					if (body.isActive() && (!body.isStaticOrKinematicObject())) {
						body.predictIntegratedTransform(timeStep, predictedTrans);

						tmp.sub2(predictedTrans.origin, body.getWorldTransform(tmpTrans).origin);
						double squareMotion = tmp.length2;

						if (body.getCcdSquareMotionThreshold() != 0 && body.getCcdSquareMotionThreshold() < squareMotion) {
							BulletStats.pushProfile("CCD motion clamping");
							try {
								if (body.getCollisionShape()?.isConvex() ?? false) {
									BulletStats.gNumClampedCcdMotions++;

									_ClosestNotMeConvexResultCallback sweepResults = _ClosestNotMeConvexResultCallback(
                    body, 
                    body.getWorldTransform(tmpTrans).origin, 
                    predictedTrans.origin, 
                    getBroadphase().getOverlappingPairCache(), 
                    getDispatcher()
                  );
									//ConvexShape convexShape = (ConvexShape)body.getCollisionShape();
									SphereShape tmpSphere = SphereShape(body.getCcdSweptSphereRadius()); //btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());

									sweepResults.collisionFilterGroup = body.getBroadphaseProxy()?.collisionFilterGroup ?? -1;
									sweepResults.collisionFilterMask = body.getBroadphaseProxy()?.collisionFilterMask ?? 0;

									convexSweepTest(tmpSphere, body.getWorldTransform(tmpTrans), predictedTrans, sweepResults);
									// JAVA NOTE: added closestHitFraction test to prevent objects being stuck
									if (sweepResults.hasHit() && (sweepResults.closestHitFraction > 0.0001)) {
										body.setHitFraction(sweepResults.closestHitFraction);
										body.predictIntegratedTransform(timeStep * body.getHitFraction(), predictedTrans);
										body.setHitFraction(0);
										//System.out.printf("clamped integration to hit fraction = %f\n", sweepResults.closestHitFraction);
									}
								}
							}
							finally {
								BulletStats.popProfile();
							}
						}

						body.proceedToTransform(predictedTrans);
					}
				}
			}
		}
		finally {
			BulletStats.popProfile();
		}
	}
	
	void predictUnconstraintMotion(double timeStep) {
		BulletStats.pushProfile("predictUnconstraintMotion");
		try {
			Transform tmpTrans = Transform();
			
			for (int i = 0; i < collisionObjects.size; i++) {
				CollisionObject? colObj = collisionObjects.getQuick(i);
				RigidBody? body = RigidBody.upcast(colObj);
				if (body != null) {
					if (!body.isStaticOrKinematicObject()) {
						if (body.isActive()) {
							body.integrateVelocities(timeStep);
							// damping
							body.applyDamping(timeStep);

							body.predictIntegratedTransform(timeStep, body.getInterpolationWorldTransform(tmpTrans));
						}
					}
				}
			}
		}
		finally {
			BulletStats.popProfile();
		}
	}
	
	void startProfiling(double timeStep) {
		//#ifndef BT_NO_PROFILE
        CProfileManager.reset();
		//#endif //BT_NO_PROFILE
	}

	void debugDrawSphere(double radius, Transform transform, Vector3 color) {
		Vector3 start = Vector3.copy(transform.origin);

		Vector3 xoffs = Vector3.zero();
		xoffs.setValues(radius, 0, 0);
		transform.basis.transform(xoffs);
		Vector3 yoffs = Vector3.zero();
		yoffs.setValues(0, radius, 0);
		transform.basis.transform(yoffs);
		Vector3 zoffs = Vector3.zero();
		zoffs.setValues(0, 0, radius);
		transform.basis.transform(zoffs);

		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		// XY
		tmp1.sub2(start, xoffs);
		tmp2.add2(start, yoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.add2(start, yoffs);
		tmp2.add2(start, xoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.add2(start, xoffs);
		tmp2.sub2(start, yoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.sub2(start, yoffs);
		tmp2.sub2(start, xoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);

		// XZ
		tmp1.sub2(start, xoffs);
		tmp2.add2(start, zoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.add2(start, zoffs);
		tmp2.add2(start, xoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.add2(start, xoffs);
		tmp2.sub2(start, zoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.sub2(start, zoffs);
		tmp2.sub2(start, xoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);

		// YZ
		tmp1.sub2(start, yoffs);
		tmp2.add2(start, zoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.add2(start, zoffs);
		tmp2.add2(start, yoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.add2(start, yoffs);
		tmp2.sub2(start, zoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
		tmp1.sub2(start, zoffs);
		tmp2.sub2(start, yoffs);
		getDebugDrawer()?.drawLine(tmp1, tmp2, color);
	}
	
	void debugDrawObject(Transform? worldTransform, CollisionShape? shape, Vector3 color) {
    if(worldTransform == null) return;
		Vector3 tmp = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		// Draw a small simplex at the center of the object
		{
			Vector3 start = Vector3.copy(worldTransform.origin);

			tmp.setValues(1, 0, 0);
			worldTransform.basis.transform(tmp);
			tmp.add(start);
			tmp2.setValues(1, 0, 0);
			getDebugDrawer()?.drawLine(start, tmp, tmp2);

			tmp.setValues(0, 1, 0);
			worldTransform.basis.transform(tmp);
			tmp.add(start);
			tmp2.setValues(0, 1, 0);
			getDebugDrawer()?.drawLine(start, tmp, tmp2);

			tmp.setValues(0, 0, 1);
			worldTransform.basis.transform(tmp);
			tmp.add(start);
			tmp2.setValues(0, 0, 1);
			getDebugDrawer()?.drawLine(start, tmp, tmp2);
		}
	}
	
	@override
	void setConstraintSolver(ConstraintSolver solver) {
		if (ownsConstraintSolver) {
			//btAlignedFree( m_constraintSolver);
		}
		ownsConstraintSolver = false;
		constraintSolver = solver;
	}

	@override
	ConstraintSolver getConstraintSolver() {
		return constraintSolver;
	}

	@override
	int getNumConstraints() {
		return constraints.size;
	}

	@override
	TypedConstraint? getConstraint(int index) {
		return constraints.getQuick(index);
	}

	// JAVA NOTE: not part of the original api
	@override
	int getNumActions() {
		return actions.size;
	}

	// JAVA NOTE: not part of the original api
	@override
	ActionInterface? getAction(int index) {
		return actions.getQuick(index);
	}

	SimulationIslandManager getSimulationIslandManager() {
		return islandManager;
	}

	CollisionWorld getCollisionWorld() {
		return this;
	}

	@override 
	DynamicsWorldType getWorldType() {
		return DynamicsWorldType.discrete;
	}

	void setNumTasks(int numTasks) {}

  void setPreTickCallback(InternalTickCallback callback){
    preTickCallback = callback;
  }

	static final Comparator<TypedConstraint?> _sortConstraintOnIslandPredicate = (TypedConstraint? lhs, TypedConstraint? rhs) {
    if(lhs == null || rhs == null) return -1;
			int rIslandId0, lIslandId0;
			rIslandId0 = _getConstraintIslandId(rhs);
			lIslandId0 = _getConstraintIslandId(lhs);
			return lIslandId0 < rIslandId0? -1 : 1;
		};
}


class _InplaceSolverIslandCallback extends IslandCallback {
  ContactSolverInfo? solverInfo;
  ConstraintSolver? solver;
  ObjectArrayList<TypedConstraint>? sortedConstraints;
  int numConstraints = 0;
  IDebugDraw? debugDrawer;
  //StackAlloc* m_stackAlloc;
  Dispatcher? dispatcher;

  void init(ContactSolverInfo? solverInfo, ConstraintSolver? solver, ObjectArrayList<TypedConstraint>? sortedConstraints, int numConstraints, IDebugDraw? debugDrawer, Dispatcher? dispatcher) {
    this.solverInfo = solverInfo;
    this.solver = solver;
    this.sortedConstraints = sortedConstraints;
    this.numConstraints = numConstraints;
    this.debugDrawer = debugDrawer;
    this.dispatcher = dispatcher;
  }
  @override
  void processIsland(ObjectArrayList<CollisionObject> bodies, int numBodies, ObjectArrayList<PersistentManifold> manifolds, int manifoldsOffset, int numManifolds, int islandId) {
    if (islandId < 0) {
      // we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
      solver?.solveGroup(bodies, numBodies, manifolds, manifoldsOffset, numManifolds, sortedConstraints, 0, numConstraints, solverInfo, debugDrawer/*,m_stackAlloc*/, dispatcher);
    }
    else {
      // also add all non-contact constraints/joints for this island
      //List<TypedConstraint> startConstraint = null;
      int startConstraintIdx = -1;
      int numCurConstraints = 0;
      int i;

      // find the first constraint for this island
      for (i = 0; i < numConstraints; i++) {
        if (DiscreteDynamicsWorld._getConstraintIslandId(sortedConstraints![i]!) == islandId) {
          //startConstraint = &m_sortedConstraints[i];
          //startConstraint = sortedConstraints.subList(i, sortedConstraints.length);
          startConstraintIdx = i;
          break;
        }
      }
      // count the number of constraints in this island
      for (; i < numConstraints; i++) {
        if (DiscreteDynamicsWorld._getConstraintIslandId(sortedConstraints![i]) == islandId) {
          numCurConstraints++;
        }
      }

      // only call solveGroup if there is some work: avoid virtual function call, its overhead can be excessive
      if ((numManifolds + numCurConstraints) > 0) {
        solver?.solveGroup(bodies, numBodies, manifolds, manifoldsOffset, numManifolds, sortedConstraints, startConstraintIdx, numCurConstraints, solverInfo, debugDrawer/*,m_stackAlloc*/, dispatcher);
      }
    }
  }
}

class _ClosestNotMeConvexResultCallback extends ClosestConvexResultCallback {
  late CollisionObject _me;
  double _allowedPenetration = 0;
  late OverlappingPairCache? _pairCache;
  late Dispatcher? _dispatcher;

  _ClosestNotMeConvexResultCallback(CollisionObject me, super.fromA, super.toA, OverlappingPairCache? pairCache, Dispatcher? dispatcher) {
    _me = me;
    _pairCache = pairCache;
    _dispatcher = dispatcher;
  }

  @override
  double addSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace) {
    if (convexResult.hitCollisionObject == _me) {
      return 1;
    }

    Vector3 linVelA = Vector3.zero(), linVelB = Vector3.zero();
    linVelA.sub2(convexToWorld, convexFromWorld);
    linVelB.setValues(0, 0, 0);//toB.getOrigin()-fromB.getOrigin();

    Vector3 relativeVelocity = Vector3.zero();
    relativeVelocity.sub2(linVelA, linVelB);
    // don't report time of impact for motion away from the contact normal (or causes minor penetration)
    if (convexResult.hitNormalLocal.dot(relativeVelocity) >= -_allowedPenetration) {
      return 1;
    }

    return super.addSingleResult(convexResult, normalInWorldSpace);
  }

  @override
  bool needsCollision(BroadphaseProxy? proxy0) {
    // don't collide with itself
    if (proxy0?.clientObject == _me) {
      return false;
    }

    // don't do CCD when the collision filters are not matching
    if (!super.needsCollision(proxy0)) {
      return false;
    }

    CollisionObject? otherObj = proxy0?.clientObject as CollisionObject;

    // call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
    if (_dispatcher?.needsResponse(_me, otherObj) ?? false) {
      // don't do CCD when there are already contact points (touching contact/penetration)
      ObjectArrayList<PersistentManifold> manifoldArray = ObjectArrayList();//List<PersistentManifold>();
      BroadphasePair? collisionPair = _pairCache?.findPair(_me.getBroadphaseHandle(), proxy0);
      if (collisionPair != null) {
        if (collisionPair.algorithm != null) {
          //manifoldArray.resize(0);
          collisionPair.algorithm?.getAllContactManifolds(manifoldArray);
          for (int j=0; j<manifoldArray.size; j++) {
            PersistentManifold? manifold = manifoldArray.getQuick(j);
            if ((manifold?.getNumContacts() ?? 0) > 0) {
              return false;
            }
          }
        }
      }
    }
    return true;
  }
}