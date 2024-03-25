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

import "package:bullet_physics/collision/dispatch/collision_flags.dart";
import "package:bullet_physics/collision/dispatch/collision_object_type.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/collision/broadphase/broadphase_proxy.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/dynamics/rigid_body_construction_info.dart";
import "package:bullet_physics/linearmath/motion_state.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/transform_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

class RigidBody extends CollisionObject {

	static final double _maxAngvel = BulletGlobals.simdHalfPi;
	
	final Matrix3 _invInertiaTensorWorld = Matrix3.zero();
	final Vector3 _linearVelocity = Vector3.zero();
	final Vector3 _angularVelocity = Vector3.zero();
	double _inverseMass = 0;
	double _angularFactor = 0;

	final Vector3 _gravity = Vector3.zero();
	final Vector3 _invInertiaLocal = Vector3.zero();
	final Vector3 _totalForce = Vector3.zero();
	final Vector3 _totalTorque = Vector3.zero();
	
	late double _linearDamping;
	late double _angularDamping;

	late bool _additionalDamping;
	late double _additionalDampingFactor;
	late double _additionalLinearDampingThresholdSqr;
	late double _additionalAngularDampingThresholdSqr;
	//late double _additionalAngularDampingFactor;

	late double _linearSleepingThreshold;
	late double _angularSleepingThreshold;

	// optionalMotionState allows to automatic synchronize the world transform for active objects
	MotionState? _optionalMotionState;

	// keep track of typed constraints referencing this rigid body
	final ObjectArrayList<TypedConstraint> _constraintRefs = ObjectArrayList();//List<TypedConstraint>();

	// for experimental overriding of friction/contact solver func
	late int contactSolverType;
	late int frictionSolverType;
	
	static int _uniqueId = 0;
	late int debugBodyId;
	
	RigidBody.constructor(RigidBodyConstructionInfo constructionInfo) {
		_setupRigidBody(constructionInfo);
	}
	
	RigidBody([double mass = 0, MotionState? motionState, CollisionShape? collisionShape, Vector3? localInertia]) {
    localInertia ??= Vector3(0, 0, 0);
		RigidBodyConstructionInfo cinfo = RigidBodyConstructionInfo(mass, motionState, collisionShape, localInertia);
		_setupRigidBody(cinfo);
	}
	
	void _setupRigidBody(RigidBodyConstructionInfo constructionInfo) {
		internalType = CollisionObjectType.rigidBody;
		
		_linearVelocity.setValues(0, 0, 0);
		_angularVelocity.setValues(0, 0, 0);
		_angularFactor = 1;
		_gravity.setValues(0, 0, 0);
		_totalForce.setValues(0, 0, 0);
		_totalTorque.setValues(0, 0, 0);
		_linearDamping = 0;
		_angularDamping = 0.5;
		_linearSleepingThreshold = constructionInfo.linearSleepingThreshold;
		_angularSleepingThreshold = constructionInfo.angularSleepingThreshold;
		_optionalMotionState = constructionInfo.motionState;
		contactSolverType = 0;
		frictionSolverType = 0;
		_additionalDamping = constructionInfo.additionalDamping;
		_additionalDampingFactor = constructionInfo.additionalDampingFactor;
		_additionalLinearDampingThresholdSqr = constructionInfo.additionalLinearDampingThresholdSqr;
		_additionalAngularDampingThresholdSqr = constructionInfo.additionalAngularDampingThresholdSqr;
		//_additionalAngularDampingFactor = constructionInfo.additionalAngularDampingFactor;

		if (_optionalMotionState != null){
			_optionalMotionState?.getWorldTransform(worldTransform);
		} 
    else{
			worldTransform.copy(constructionInfo.startWorldTransform);
		}

		interpolationWorldTransform.copy(worldTransform);
		interpolationLinearVelocity.setValues(0, 0, 0);
		interpolationAngularVelocity.setValues(0, 0, 0);

		// moved to CollisionObject
		friction = constructionInfo.friction;
		restitution = constructionInfo.restitution;

		setCollisionShape(constructionInfo.collisionShape);
		debugBodyId = _uniqueId++;

		setMassProps(constructionInfo.mass, constructionInfo.localInertia);
		setDamping(constructionInfo.linearDamping, constructionInfo.angularDamping);
		updateInertiaTensor();
	}
	
	void destroy() {
		// No constraints should point to this rigidbody
		// Remove constraints from the dynamics world before you delete the related rigidbodies. 
		assert (_constraintRefs.isEmpty);
	}

	void proceedToTransform(Transform newTrans) {
		setCenterOfMassTransform(newTrans);
	}
	
	/**
	 * To keep collision detection and dynamics separate we don't store a rigidbody pointer,
	 * but a rigidbody is derived from CollisionObject, so we can safely perform an upcast.
	 */
	static RigidBody? upcast(CollisionObject? colObj) {
		if (colObj?.getInternalType() == CollisionObjectType.rigidBody) {
			return colObj as RigidBody;
		}
		return null;
	}

	/**
	 * Continuous collision detection needs prediction.
	 */
	void predictIntegratedTransform(double timeStep, Transform predictedTransform) {
		TransformUtil.integrateTransform(worldTransform, _linearVelocity, _angularVelocity, timeStep, predictedTransform);
	}
	
	void saveKinematicState(double timeStep) {
		//todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
		if (timeStep != 0) {
			//if we use motionstate to synchronize world transforms, get the kinematic/animated world transform
			if (getMotionState() != null) {
				getMotionState()?.getWorldTransform(worldTransform);
			}
			//Vector3 linVel = Vector3.zero(), angVel = Vector3.zero();

			TransformUtil.calculateVelocity(interpolationWorldTransform, worldTransform, timeStep, _linearVelocity, _angularVelocity);
			interpolationLinearVelocity.setFrom(_linearVelocity);
			interpolationAngularVelocity.setFrom(_angularVelocity);
			interpolationWorldTransform.copy(worldTransform);
		//printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
		}
	}
	
	void applyGravity() {
		if (isStaticOrKinematicObject()){
			return;
    }
		applyCentralForce(_gravity);
	}
	
	void setGravity(Vector3 acceleration) {
		if (_inverseMass != 0) {
			_gravity.scaleFrom(1 / _inverseMass, acceleration);
		}
	}

	Vector3 getGravity(Vector3 out) {
		out.setFrom(_gravity);
		return out;
	}

	void setDamping(double linDamping, double angDamping) {
		_linearDamping = MiscUtil.genClamped(linDamping, 0, 1);
		_angularDamping = MiscUtil.genClamped(angDamping, 0, 1);
	}

	double getLinearDamping() {
		return _linearDamping;
	}

	double getAngularDamping() {
		return _angularDamping;
	}

	double getLinearSleepingThreshold() {
		return _linearSleepingThreshold;
	}

	double getAngularSleepingThreshold() {
		return _angularSleepingThreshold;
	}

	/**
	 * Damps the velocity, using the given linearDamping and angularDamping.
	 */
	void applyDamping(double timeStep) {
		// On damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
		// todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

		//#define USE_OLD_DAMPING_METHOD 1
		//#ifdef USE_OLD_DAMPING_METHOD
		//linearVelocity.scale(MiscUtil.GEN_clamped((1 - timeStep * linearDamping), 0, 1));
		//angularVelocity.scale(MiscUtil.GEN_clamped((1 - timeStep * angularDamping), 0, 1));
		//#else
		_linearVelocity.scale(pow(1 - _linearDamping, timeStep).toDouble());
		_angularVelocity.scale(pow(1 - _angularDamping, timeStep).toDouble());
		//#endif

		if (_additionalDamping) {
			// Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
			// Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
			if ((_angularVelocity.length2 < _additionalAngularDampingThresholdSqr) &&
					(_linearVelocity.length2 < _additionalLinearDampingThresholdSqr)) {
				_angularVelocity.scale(_additionalDampingFactor);
				_linearVelocity.scale(_additionalDampingFactor);
			}

			double speed = _linearVelocity.length;
			if (speed < _linearDamping) {
				double dampVel = 0.005;
				if (speed > dampVel) {
					Vector3 dir = Vector3.copy(_linearVelocity);
					dir.normalize();
					dir.scale(dampVel);
					_linearVelocity.sub(dir);
				}
				else {
					_linearVelocity.setValues(0, 0, 0);
				}
			}

			double angSpeed = _angularVelocity.length;
			if (angSpeed < _angularDamping) {
				double angDampVel = 0.005;
				if (angSpeed > angDampVel) {
					Vector3 dir = Vector3.copy(_angularVelocity);
					dir.normalize();
					dir.scale(angDampVel);
					_angularVelocity.sub(dir);
				}
				else {
					_angularVelocity.setValues(0, 0, 0);
				}
			}
		}
	}

	void setMassProps(double mass, Vector3 inertia) {
		if (mass == 0) {
			collisionFlags |= CollisionFlags.staticObject;
			_inverseMass = 0;
		}
		else {
			collisionFlags &= (~CollisionFlags.staticObject);
			_inverseMass = 1 / mass;
		}

		_invInertiaLocal.setValues(inertia.x != 0 ? 1 / inertia.x : 0,
				inertia.y != 0 ? 1 / inertia.y : 0,
				inertia.z != 0 ? 1 / inertia.z : 0);
	}

	double getInvMass() {
		return _inverseMass;
	}

	Matrix3 getInvInertiaTensorWorld(Matrix3 out) {
		out.setFrom(_invInertiaTensorWorld);
		return out;
	}
	
	void integrateVelocities(double step) {
		if (isStaticOrKinematicObject()) {
			return;
		}

		_linearVelocity.scaleAdd(_inverseMass * step, _totalForce, _linearVelocity);
		Vector3 tmp = Vector3.copy(_totalTorque);
		_invInertiaTensorWorld.transform(tmp);
		_angularVelocity.scaleAdd(step, tmp, _angularVelocity);

		// clamp angular velocity. collision calculations will fail on higher angular velocities	
		double angvel = _angularVelocity.length;
		if (angvel * step > _maxAngvel) {
			_angularVelocity.scale((_maxAngvel / step) / angvel);
		}
	}

	void setCenterOfMassTransform(Transform xform) {
		if (isStaticOrKinematicObject()) {
			interpolationWorldTransform.copy(worldTransform);
		}
		else {
			interpolationWorldTransform.copy(xform);
		}
		getLinearVelocity(interpolationLinearVelocity);
		getAngularVelocity(interpolationAngularVelocity);
		worldTransform.copy(xform);
		updateInertiaTensor();
	}

	void applyCentralForce(Vector3 force) {
		_totalForce.add(force);
	}
	
	Vector3 getInvInertiaDiagLocal(Vector3 out) {
		out.setFrom(_invInertiaLocal);
		return out;
	}

	void setInvInertiaDiagLocal(Vector3 diagInvInertia) {
		_invInertiaLocal.setFrom(diagInvInertia);
	}

	void setSleepingThresholds(double linear, double angular) {
		_linearSleepingThreshold = linear;
		_angularSleepingThreshold = angular;
	}

	void applyTorque(Vector3 torque) {
		_totalTorque.add(torque);
	}

	void applyForce(Vector3 force, Vector3 relPos) {
		applyCentralForce(force);
		
		Vector3 tmp = Vector3.zero();
		tmp.cross2(relPos, force);
		tmp.scale(_angularFactor);
		applyTorque(tmp);
	}

	void applyCentralImpulse(Vector3 impulse) {
		_linearVelocity.scaleAdd(_inverseMass, impulse, _linearVelocity);
	}
	

	void applyTorqueImpulse(Vector3 torque) {
		Vector3 tmp = Vector3.copy(torque);
		_invInertiaTensorWorld.transform(tmp);
		_angularVelocity.add(tmp);
	}


	void applyImpulse(Vector3 impulse, Vector3 relPos) {
		if (_inverseMass != 0) {
			applyCentralImpulse(impulse);
			if (_angularFactor != 0) {
				Vector3 tmp = Vector3.zero();
				tmp.cross2(relPos, impulse);
				tmp.scale(_angularFactor);
				applyTorqueImpulse(tmp);
			}
		}
	}

	/**
	 * Optimization for the iterative solver: avoid calculating constant terms involving inertia, normal, relative position.
	 */
	void internalApplyImpulse(Vector3 linearComponent, Vector3 angularComponent, double impulseMagnitude) {
		if (_inverseMass != 0) {
			_linearVelocity.scaleAdd(impulseMagnitude, linearComponent, _linearVelocity);
			if (_angularFactor != 0) {
				_angularVelocity.scaleAdd(impulseMagnitude * _angularFactor, angularComponent, _angularVelocity);
			}
		}
	}

	void clearForces() {
		_totalForce.setValues(0, 0, 0);
		_totalTorque.setValues(0, 0, 0);
	}
	
	void updateInertiaTensor() {
		Matrix3 mat1 = Matrix3.zero();
		MatrixUtil.scale(mat1, worldTransform.basis, _invInertiaLocal);

		Matrix3 mat2 = Matrix3.copy(worldTransform.basis);
		mat2.transpose();

		_invInertiaTensorWorld.mul2(mat1, mat2);
	}
	
	Vector3 getCenterOfMassPosition(Vector3 out) {
		out.setFrom(worldTransform.origin);
		return out;
	}

	Quaternion getOrientation(Quaternion out) {
		MatrixUtil.getRotation(worldTransform.basis, out);
		return out;
	}
	
	Transform getCenterOfMassTransform(Transform out) {
		out.copy(worldTransform);
		return out;
	}

	Vector3 getLinearVelocity(Vector3 out) {
		out.setFrom(_linearVelocity);
		return out;
	}

	Vector3 getAngularVelocity(Vector3 out) {
		out.setFrom(_angularVelocity);
		return out;
	}

	void setLinearVelocity(Vector3 linVel) {
		assert (collisionFlags != CollisionFlags.staticObject);
		_linearVelocity.setFrom(linVel);
	}

	void setAngularVelocity(Vector3 angVel) {
		assert (collisionFlags != CollisionFlags.staticObject);
		_angularVelocity.setFrom(angVel);
	}

	Vector3 getVelocityInLocalPoint(Vector3 relPos, Vector3 out) {
		// we also calculate lin/ang velocity for kinematic objects
		Vector3 vec = out;
		vec.cross2(_angularVelocity, relPos);
		vec.add(_linearVelocity);
		return out;

		//for kinematic objects, we could also use use:
		//		return 	(m_worldTransform(relPos) - m_interpolationWorldTransform(relPos)) / m_kinematicTimeStep;
	}

	void translate(Vector3 v) {
		worldTransform.origin.add(v);
	}
	
	void getAabb(Vector3 aabbMin, Vector3 aabbMax) {
		getCollisionShape()?.getAabb(worldTransform, aabbMin, aabbMax);
	}

	double computeImpulseDenominator(Vector3 pos, Vector3 normal) {
		Vector3 r0 = Vector3.zero();
		r0.sub2(pos, getCenterOfMassPosition(Vector3.zero()));

		Vector3 c0 = Vector3.zero();
		c0.cross2(r0, normal);

		Vector3 tmp = Vector3.zero();
		MatrixUtil.transposeTransform(tmp, c0, getInvInertiaTensorWorld(Matrix3.zero()));

		Vector3 vec = Vector3.zero();
		vec.cross2(tmp, r0);

		return _inverseMass + normal.dot(vec);
	}

	double computeAngularImpulseDenominator(Vector3 axis) {
		Vector3 vec = Vector3.zero();
		MatrixUtil.transposeTransform(vec, axis, getInvInertiaTensorWorld(Matrix3.zero()));
		return axis.dot(vec);
	}

	void updateDeactivation(double timeStep) {
		if ((getActivationState() == CollisionObject.islandSleeping) || (getActivationState() == CollisionObject.disableDeactivation)) {
			return;
		}

		if ((getLinearVelocity(Vector3.zero()).length2 < _linearSleepingThreshold * _linearSleepingThreshold) &&
				(getAngularVelocity(Vector3.zero()).length2 < _angularSleepingThreshold * _angularSleepingThreshold)) {
			deactivationTime = (deactivationTime ?? 0) + timeStep;
		}
		else {
			deactivationTime = 0;
			setActivationState(0);
		}
	}

	bool wantsSleeping() {
		if (getActivationState() == CollisionObject.disableDeactivation) {
			return false;
		}

		// disable deactivation
		if (BulletGlobals.isDeactivationDisabled() || (BulletGlobals.getDeactivationTime() == 0)) {
			return false;
		}

		if ((getActivationState() == CollisionObject.islandSleeping) || (getActivationState() == CollisionObject.wantsDeactivation)) {
			return true;
		}

		if ((deactivationTime ?? 0) > BulletGlobals.getDeactivationTime()) {
			return true;
		}
		return false;
	}
	
	BroadphaseProxy? getBroadphaseProxy() {
		return broadphaseHandle;
	}

	void setNewBroadphaseProxy(BroadphaseProxy broadphaseProxy) {
		broadphaseHandle = broadphaseProxy;
	}

	MotionState? getMotionState() {
		return _optionalMotionState;
	}

	void setMotionState(MotionState motionState) {
		_optionalMotionState = motionState;
		if (_optionalMotionState != null) {
			motionState.getWorldTransform(worldTransform);
		}
	}

	void setAngularFactor(double angFac) {
		_angularFactor = angFac;
	}

	double getAngularFactor() {
		return _angularFactor;
	}

	/**
	 * Is this rigidbody added to a CollisionWorld/DynamicsWorld/Broadphase?
	 */
	bool isInWorld() {
		return (getBroadphaseProxy() != null);
	}

	@override
	bool checkCollideWithOverride(CollisionObject co) {
		// TODO: change to cast
		RigidBody? otherRb = RigidBody.upcast(co);
		if (otherRb == null) {
			return true;
		}

		for (int i = 0; i < _constraintRefs.size; ++i) {
			TypedConstraint? c = _constraintRefs.getQuick(i);
			if (c?.getRigidBodyA() == otherRb || c?.getRigidBodyB() == otherRb) {
				return false;
			}
		}

		return true;
	}

	void addConstraintRef(TypedConstraint c) {
		int index = _constraintRefs.indexOf(c);
		if (index == -1) {
			_constraintRefs.add(c);
		}

		allowCheckCollideWith = true;
	}
	
	void removeConstraintRef(TypedConstraint c) {
		_constraintRefs.remove(c);
		allowCheckCollideWith = _constraintRefs.isNotEmpty;
	}

	TypedConstraint? getConstraintRef(int index) {
		return _constraintRefs.getQuick(index);
	}

	int getNumConstraintRefs() {
		return _constraintRefs.size;
	}
	
}
