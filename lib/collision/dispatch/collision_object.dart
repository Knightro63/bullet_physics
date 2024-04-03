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

import 'package:bullet_physics/collision/broadphase/broadphase_proxy.dart';
import "package:bullet_physics/collision/dispatch/collision_flags.dart";
import 'package:bullet_physics/collision/dispatch/collision_object_type.dart';
import 'package:bullet_physics/collision/shapes/collision_shape.dart';
import 'package:bullet_physics/linearmath/transform.dart';
import 'package:vector_math/vector_math.dart';

class CollisionObject {
	static const int activeTag = 1;
	static const int islandSleeping = 2;
	static const int wantsDeactivation = 3;
	static const int disableDeactivation = 4;
	static const int disableSimulation = 5;
	Transform worldTransform = Transform();

	///m_interpolationWorldTransform is used for CCD and interpolation
	///it can be either previous or future (predicted) transform
	final Transform interpolationWorldTransform = Transform();
	//those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying velocities) 
	//without destroying the continuous interpolated motion (which uses this interpolation velocities)
	final Vector3 interpolationLinearVelocity = Vector3.zero();
	final Vector3 interpolationAngularVelocity = Vector3.zero();
	BroadphaseProxy? broadphaseHandle;
	CollisionShape? collisionShape;
	
	// rootCollisionShape is temporarily used to store the original collision shape
	// The collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
	// If it is null, the collisionShape is not temporarily replaced.
	CollisionShape? rootCollisionShape;
	
	int collisionFlags = CollisionFlags.staticObject;
	int islandTag1 = -1;
	int companionId = -1;
	int activationState1 = 1;
	double deactivationTime = 0;
	double friction = 0.5;
	double restitution = 0;

	///users can point to their objects, m_userPointer is not used by Bullet, see setUserPointer/getUserPointer
	Object? userObjectPointer;

	// internalType is reserved to distinguish Bullet's CollisionObject, RigidBody, SoftBody etc.
	// do not assign your own internalType unless you write a dynamics object class.
	CollisionObjectType internalType = CollisionObjectType.collisionObject;

	///time of impact calculation
	double hitFraction = 1;
	///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
	double ccdSweptSphereRadius = 0;

	/// Don't do continuous collision detection if the motion (in one step) is less then ccdMotionThreshold
	double ccdMotionThreshold = 0;
	/// If some object should have elaborate collision filtering by sub-classes
	bool allowCheckCollideWith = true;

  CollisionObject();

	bool checkCollideWithOverride(CollisionObject co) {
		return true;
	}

	bool mergesSimulationIslands() {
		///static objects, kinematic and object without contact response don't merge islands
		return ((collisionFlags & (CollisionFlags.staticObject | CollisionFlags.kinematicObject | CollisionFlags.noContactResponse)) == 0);
	}

	bool isStaticObject() {
		return (collisionFlags & CollisionFlags.staticObject) != 0;
	}

	bool isKinematicObject() {
		return (collisionFlags & CollisionFlags.kinematicObject) != 0;
	}

	bool isStaticOrKinematicObject() {
		return (collisionFlags & (CollisionFlags.kinematicObject | CollisionFlags.staticObject)) != 0;
	}

	bool hasContactResponse() {
		return (collisionFlags & CollisionFlags.noContactResponse) == 0;
	}

	CollisionShape? getCollisionShape() {
		return collisionShape;
	}

	void setCollisionShape(CollisionShape? collisionShape) {
		this.collisionShape = collisionShape;
		rootCollisionShape = collisionShape;
	}

	CollisionShape? getRootCollisionShape() {
		return rootCollisionShape;
	}

	/**
	 * Avoid using this internal API call.
	 * internalSetTemporaryCollisionShape is used to temporary replace the actual collision shape by a child collision shape.
	 */
	void internalSetTemporaryCollisionShape(CollisionShape? collisionShape) {
		this.collisionShape = collisionShape;
	}

	int getActivationState() {
		return activationState1;
	}

	void setActivationState(int newState) {
		if ((activationState1 != disableDeactivation) && (activationState1 != disableSimulation)) {
			activationState1 = newState;
		}
	}

	double getDeactivationTime() {
		return deactivationTime;
	}

	void setDeactivationTime(double deactivationTime) {
		this.deactivationTime = deactivationTime;
	}

	void forceActivationState(int newState) {
		activationState1 = newState;
	}

	void activate([bool forceActivation = false]) {
		if (forceActivation || (collisionFlags & (CollisionFlags.staticObject | CollisionFlags.kinematicObject)) == 0) {
			setActivationState(activeTag);
			deactivationTime = 0;
		}
	}

	bool isActive() {
		return ((getActivationState() != islandSleeping) && (getActivationState() != disableSimulation));
	}

	double getRestitution() {
		return restitution;
	}

	void setRestitution(double restitution) {
		this.restitution = restitution;
	}

	double getFriction() {
		return friction;
	}

	void setFriction(double friction) {
		this.friction = friction;
	}

	// reserved for Bullet internal usage
	CollisionObjectType getInternalType() {
		return internalType;
	}

	Transform getWorldTransform(Transform out) {
		out.copy(worldTransform);
		return out;
	}

	void setWorldTransform(Transform? worldTransform) {
    if(worldTransform != null){
		  this.worldTransform.copy(worldTransform);
    }
	}

	BroadphaseProxy? getBroadphaseHandle() {
		return broadphaseHandle;
	}

	void setBroadphaseHandle(BroadphaseProxy? broadphaseHandle) {
		this.broadphaseHandle = broadphaseHandle;
	}

	Transform getInterpolationWorldTransform(Transform out) {
		out.copy(interpolationWorldTransform);
		return out;
	}

	void setInterpolationWorldTransform(Transform interpolationWorldTransform) {
		this.interpolationWorldTransform.copy(interpolationWorldTransform);
	}

	void setInterpolationLinearVelocity(Vector3 linvel) {
		interpolationLinearVelocity.setFrom(linvel);
	}

	void setInterpolationAngularVelocity(Vector3 angvel) {
		interpolationAngularVelocity.setFrom(angvel);
	}

	Vector3 getInterpolationLinearVelocity(Vector3 out) {
		out.setFrom(interpolationLinearVelocity);
		return out;
	}

	Vector3 getInterpolationAngularVelocity(Vector3 out) {
		out.setFrom(interpolationAngularVelocity);
		return out;
	}

	int getIslandTag() {
		return islandTag1;
	}

	void setIslandTag(int islandTag) {
		this.islandTag1 = islandTag;
	}

	int getCompanionId() {
		return companionId;
	}

	void setCompanionId(int companionId) {
		this.companionId = companionId;
	}

	double getHitFraction() {
		return hitFraction;
	}

	void setHitFraction(double hitFraction) {
		this.hitFraction = hitFraction;
	}

	int getCollisionFlags() {
		return collisionFlags;
	}

	void setCollisionFlags(int collisionFlags) {
		this.collisionFlags = collisionFlags;
	}

	// Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
	double getCcdSweptSphereRadius() {
		return ccdSweptSphereRadius;
	}

	// Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm::
	void setCcdSweptSphereRadius(double ccdSweptSphereRadius) {
		this.ccdSweptSphereRadius = ccdSweptSphereRadius;
	}

	double getCcdMotionThreshold() {
		return ccdMotionThreshold;
	}

	double getCcdSquareMotionThreshold() {
		return ccdMotionThreshold * ccdMotionThreshold;
	}

	// Don't do continuous collision detection if the motion (in one step) is less then ccdMotionThreshold
	void setCcdMotionThreshold(double ccdMotionThreshold) {
		this.ccdMotionThreshold = ccdMotionThreshold;
	}

	Object? getUserPointer() {
		return userObjectPointer;
	}

	void setUserPointer(Object userObjectPointer) {
		this.userObjectPointer = userObjectPointer;
	}

	bool checkCollideWith(CollisionObject co) {
		if (allowCheckCollideWith) {
			return checkCollideWithOverride(co);
		}

		return true;
	}
}
