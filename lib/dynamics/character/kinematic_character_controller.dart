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

import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/collision_world.dart";
import "package:bullet_physics/collision/dispatch/pair_caching_ghost_object.dart";
import "package:bullet_physics/collision/narrowphase/manifold_point.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/collision/broadphase/broadphase_pair.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/dynamics/action_interface.dart";
import "package:bullet_physics/linearmath/i_debug_draw.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

/**
 * KinematicCharacterController is an object that supports a sliding motion in
 * a world. It uses a {@link GhostObject} and convex sweep test to test for upcoming
 * collisions. This is combined with discrete collision detection to recover
 * from penetrations.<p>
 *
 * Interaction between KinematicCharacterController and dynamic rigid bodies
 * needs to be explicity implemented by the user.
 * 
 * @author tomrbryn
 */
class KinematicCharacterController extends ActionInterface {

	final List<Vector3> _upAxisDirection = [
		Vector3(1.0, 0.0, 0.0),
		Vector3(0.0, 1.0, 0.0),
		Vector3(0.0, 0.0, 1.0),
  ];

	double halfHeight = 0;
	
	PairCachingGhostObject ghostObject;

	// is also in ghostObject, but it needs to be convex, so we store it here
	// to avoid upcast
	ConvexShape convexShape;

	double verticalVelocity = 0;
	double verticalOffset = 0;
	
	double fallSpeed = 55;
	double jumpSpeed = 10;
	double maxJumpHeight = 0;
	
	double maxSlopeRadians = 0; // Slope angle that is set (used for returning the exact value) 
	double maxSlopeCosine = 0; // Cosine equivalent of m_maxSlopeRadians (calculated once when set, for optimization)

	double gravity = 9.8;
	
	double turnAngle = 0;

	double stepHeight = 0;

	double addedMargin = 0.02; // @todo: remove this and fix the code

	// this is the desired walk direction, set by the user
	Vector3 walkDirection = Vector3.zero();
	Vector3 normalizedDirection = Vector3.zero();

	// some internal variables
	Vector3 currentPosition = Vector3.zero();
	double currentStepOffset = 0;
	Vector3 targetPosition = Vector3.zero();

	// keep track of the contact manifolds
	ObjectArrayList<PersistentManifold> manifoldArray = ObjectArrayList();

	bool touchingContact = false;
	Vector3 touchingNormal = Vector3.zero();

	bool wasOnGround = false;
	bool wasJumping = false;
	
	bool useGhostObjectSweepTest = true;
	bool useWalkDirection = true;
	double velocityTimeInterval = 0;
	int upAxis;

	CollisionObject? me;

	KinematicCharacterController(this.ghostObject, this.convexShape, this.stepHeight, [this.upAxis = 1]) {
		setMaxSlope(((50.0/180.0) * pi));
	}

	PairCachingGhostObject _getGhostObject() {
		return ghostObject;
	}

	// ActionInterface interface
  @override
	void updateAction(CollisionWorld collisionWorld, double deltaTime) {
		preStep(collisionWorld);
		playerStep(collisionWorld, deltaTime);
	}

	// ActionInterface interface
  @override
	void debugDraw(IDebugDraw? debugDrawer) {}

	void setUpAxis(int axis) {
		if (axis < 0) {
			axis = 0;
		}
		if (axis > 2) {
			axis = 2;
		}
		upAxis = axis;
	}

	/**
	 * This should probably be called setPositionIncrementPerSimulatorStep. This
	 * is neither a direction nor a velocity, but the amount to increment the
	 * position each simulation iteration, regardless of dt.<p>
	 *
	 * This call will reset any velocity set by {@link #setVelocityForTimeInterval}.
	 */
	void	setWalkDirection(Vector3 walkDirection) {
		useWalkDirection = true;
		this.walkDirection.setFrom(walkDirection);
		normalizedDirection.setFrom(_getNormalizedVector(walkDirection, Vector3.zero()));
	}

	/**
	 * Caller provides a velocity with which the character should move for the
	 * given time period. After the time period, velocity is reset to zero.
	 * This call will reset any walk direction set by {@link #setWalkDirection}.
	 * Negative time intervals will result in no motion.
	 */
	void setVelocityForTimeInterval(Vector3 velocity, double timeInterval) {
		useWalkDirection = false;
		walkDirection.setFrom(velocity);
		normalizedDirection.setFrom(_getNormalizedVector(walkDirection, Vector3.zero()));
		velocityTimeInterval = timeInterval;
	}

	void reset() {}

	void warp(Vector3 origin) {
		Transform xform = Transform();
		xform.setIdentity();
		xform.origin.setFrom(origin);
		ghostObject.setWorldTransform(xform);
	}

	void preStep(CollisionWorld collisionWorld) {
		int numPenetrationLoops = 0;
		touchingContact = false;
		while (recoverFromPenetration(collisionWorld)) {
			numPenetrationLoops++;
			touchingContact = true;
			if (numPenetrationLoops > 4) {
				//printf("character could not recover from penetration = %d\n", numPenetrationLoops);
				break;
			}
		}

		currentPosition.setFrom(ghostObject.getWorldTransform(Transform()).origin);
		targetPosition.setFrom(currentPosition);
		//printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);
	}
	
	void playerStep(CollisionWorld collisionWorld, double dt) {
		//printf("playerStep(): ");
		//printf("  dt = %f", dt);

		// quick check...
		if (!useWalkDirection && velocityTimeInterval <= 0.0) {
			//printf("\n");
			return; // no motion
		}
		
		wasOnGround = onGround();
		
		// Update fall velocity.
		verticalVelocity -= gravity * dt;
		if(verticalVelocity > 0.0 && verticalVelocity > jumpSpeed)
		{
			verticalVelocity = jumpSpeed;
		}
		if(verticalVelocity < 0.0 && verticalVelocity.abs() > fallSpeed.abs())
		{
			verticalVelocity = -fallSpeed.abs();
		}
		verticalOffset = verticalVelocity * dt;

		Transform xform = ghostObject.getWorldTransform(Transform());

		//printf("walkDirection(%f,%f,%f)\n",walkDirection[0],walkDirection[1],walkDirection[2]);
		//printf("walkSpeed=%f\n",walkSpeed);

		stepUp(collisionWorld);
		if (useWalkDirection) {
			//System.out.println("playerStep 3");
			stepForwardAndStrafe(collisionWorld, walkDirection);
		}
		else {
			print("playerStep 4");
			//printf("  time: %f", m_velocityTimeInterval);

			// still have some time left for moving!
			double dtMoving = (dt < velocityTimeInterval) ? dt : velocityTimeInterval;
			velocityTimeInterval -= dt;

			// how far will we move while we are moving?
			Vector3 move = Vector3.zero();
			move.scaleFrom(dtMoving, walkDirection);

			//printf("  dtMoving: %f", dtMoving);

			// okay, step
			stepForwardAndStrafe(collisionWorld, move);
		}
		stepDown(collisionWorld, dt);

		//printf("\n");

		xform.origin.setFrom(currentPosition);
		ghostObject.setWorldTransform(xform);
	}

	void setFallSpeed(double fallSpeed) {
		this.fallSpeed = fallSpeed;
	}
	
	void setJumpSpeed(double jumpSpeed) {
		this.jumpSpeed = jumpSpeed;
	}

	void setMaxJumpHeight(double maxJumpHeight) {
		this.maxJumpHeight = maxJumpHeight;
	}
	
	bool canJump() {
		return onGround();
	}
	
	void jump() {
		if (!canJump()) return;
		
		verticalVelocity = jumpSpeed;
    wasJumping = true;

		//#if 0
		//currently no jumping.
		//btTransform xform;
		//m_rigidBody->getMotionState()->getWorldTransform (xform);
		//btVector3 up = xform.getBasis()[1];
		//up.normalize ();
		//btScalar magnitude = (btScalar(1.0)/m_rigidBody->getInvMass()) * btScalar(8.0);
		//m_rigidBody->applyCentralImpulse (up * magnitude);
		//#endif
	}
	
	void setGravity(double gravity) {
		this.gravity = gravity;
	}
	
	double getGravity() {
		return gravity;
	}
	
	void setMaxSlope(double slopeRadians) {
		maxSlopeRadians = slopeRadians;
		maxSlopeCosine = cos(slopeRadians);
	}
	
	double getMaxSlope() {
		return maxSlopeRadians;
	}
	
	bool onGround() {
		return verticalVelocity == 0.0 && verticalOffset == 0.0;
	}

	// static helper method
	static Vector3 _getNormalizedVector(Vector3 v, Vector3 out) {
		out.setFrom(v);
		out.normalize();
		if (out.length < BulletGlobals.simdEpsilon) {
			out.setValues(0, 0, 0);
		}
		return out;
	}

	/**
	 * Returns the reflection direction of a ray going 'direction' hitting a surface
	 * with normal 'normal'.<p>
	 *
	 * From: http://www-cs-students.stanford.edu/~adityagp/final/node3.html
	 */
	Vector3 computeReflectionDirection(Vector3 direction, Vector3 normal, Vector3 out) {
		// return direction - (btScalar(2.0) * direction.dot(normal)) * normal;
		out.setFrom(normal);
		out.scale(-2.0 * direction.dot(normal));
		out.add(direction);
		return out;
	}

	/**
	 * Returns the portion of 'direction' that is parallel to 'normal'
	 */
	Vector3 parallelComponent(Vector3 direction, Vector3 normal, Vector3 out) {
		//btScalar magnitude = direction.dot(normal);
		//return normal * magnitude;
		out.setFrom(normal);
		out.scale(direction.dot(normal));
		return out;
	}

	/**
	 * Returns the portion of 'direction' that is perpindicular to 'normal'
	 */
	Vector3 perpindicularComponent(Vector3 direction, Vector3 normal, Vector3 out) {
		//return direction - parallelComponent(direction, normal);
		Vector3 perpendicular = parallelComponent(direction, normal, out);
		perpendicular.scale(-1);
		perpendicular.add(direction);
		return perpendicular;
	}

	bool recoverFromPenetration(CollisionWorld collisionWorld) {
		bool penetration = false;

		collisionWorld.getDispatcher()?.dispatchAllCollisionPairs(ghostObject.getOverlappingPairCache(), collisionWorld.getDispatchInfo(), collisionWorld.getDispatcher());

		currentPosition.setFrom(ghostObject.getWorldTransform(Transform()).origin);

		double maxPen = 0.0;
		for (int i=0; i<ghostObject.getOverlappingPairCache().getNumOverlappingPairs(); i++) {
			manifoldArray.clear();

			BroadphasePair? collisionPair = ghostObject.getOverlappingPairCache().getOverlappingPairArray().getQuick(i);
      //XXX: added no contact response
      if (!((collisionPair?.pProxy0?.clientObject as CollisionObject?)?.hasContactResponse() ?? false) || !((collisionPair?.pProxy1?.clientObject as CollisionObject?)?.hasContactResponse() ?? false)){
        continue;
      }
			if (collisionPair?.algorithm != null) {
				collisionPair?.algorithm?.getAllContactManifolds(manifoldArray);
			}

			for (int j=0; j<manifoldArray.size; j++) {
				PersistentManifold? manifold = manifoldArray.getQuick(j);
				double directionSign = manifold?.getBody0() == ghostObject? -1.0 : 1.0;
				for (int p=0; p<(manifold?.getNumContacts() ?? 0); p++) {
					ManifoldPoint pt = manifold!.getContactPoint(p);

					double dist = pt.getDistance();
					if (dist < 0.0) {
						if (dist < maxPen) {
							maxPen = dist;
							touchingNormal.setFrom(pt.normalWorldOnB);//??
							touchingNormal.scale(directionSign);
						}

						currentPosition.scaleAdd(directionSign * dist * 0.2, pt.normalWorldOnB, currentPosition);

						penetration = true;
					}
					else {
						//printf("touching %f\n", dist);
					}
				}

				//manifold->clearManifold();
			}
		}
		
		Transform newTrans = ghostObject.getWorldTransform(Transform());
		newTrans.origin.setFrom(currentPosition);
		ghostObject.setWorldTransform(newTrans);
		//printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);

		//System.out.println("recoverFromPenetration "+penetration+" "+touchingNormal);

		return penetration;
	}
	
	void stepUp(CollisionWorld world) {
		// phase 1: up
		Transform start = Transform();
		Transform end = Transform();
		targetPosition.scaleAdd(stepHeight + (verticalOffset > 0.0?verticalOffset:0.0), _upAxisDirection[upAxis], currentPosition);

		start.setIdentity ();
		end.setIdentity ();

		/* FIXME: Handle penetration properly */
		start.origin.scaleAdd(convexShape.getMargin() + addedMargin, _upAxisDirection[upAxis], currentPosition);
		end.origin.setFrom(targetPosition);
		
		// Find only sloped/flat surface hits, avoid wall and ceiling hits...
		Vector3 up = Vector3.zero();
		up.scaleFrom(-1, _upAxisDirection[upAxis]);
		_KinematicClosestNotMeConvexResultCallback callback = _KinematicClosestNotMeConvexResultCallback(ghostObject, up, 0.7071);
		callback.collisionFilterGroup = _getGhostObject().getBroadphaseHandle()?.collisionFilterGroup ?? -1;
		callback.collisionFilterMask = _getGhostObject().getBroadphaseHandle()?.collisionFilterMask ?? 0;

		if (useGhostObjectSweepTest) {
			ghostObject.convexSweepTest(convexShape, start, end, callback, world.getDispatchInfo().allowedCcdPenetration);
		}
		else {
			world.convexSweepTest(convexShape, start, end, callback);
		}

		if (callback.hasHit()) {
      // Only modify the position if the hit was a slope and not a wall or ceiling.
      if(callback.hitNormalWorld.dot(_upAxisDirection[upAxis]) > 0.0){
        // we moved up only a fraction of the step height
        currentStepOffset = stepHeight * callback.closestHitFraction;
        currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
        verticalVelocity = 0.0;
        verticalOffset = 0.0;
      }
		}
		else {
			currentStepOffset = stepHeight;
			currentPosition.setFrom(targetPosition);
		}
	}

	void updateTargetPositionBasedOnCollision(Vector3 hitNormal, [double tangentMag = 0, double normalMag = 1]) {
		Vector3 movementDirection = Vector3.zero();
		movementDirection.sub2(targetPosition, currentPosition);
		double movementLength = movementDirection.length;
		if (movementLength>BulletGlobals.simdEpsilon) {
			movementDirection.normalize();

			Vector3 reflectDir = computeReflectionDirection(movementDirection, hitNormal, Vector3.zero());
			reflectDir.normalize();

			Vector3 parallelDir = parallelComponent(reflectDir, hitNormal, Vector3.zero());
			Vector3 perpindicularDir = perpindicularComponent(reflectDir, hitNormal, Vector3.zero());

			targetPosition.setFrom(currentPosition);
			if (false) {//tangentMag != 0.0)
				Vector3 parComponent = Vector3.zero();
				parComponent.scaleFrom(tangentMag * movementLength, parallelDir);
				//printf("parComponent=%f,%f,%f\n",parComponent[0],parComponent[1],parComponent[2]);
				targetPosition.add(parComponent);
			}

			if (normalMag != 0.0) {
				Vector3 perpComponent = Vector3.zero();
				perpComponent.scaleFrom(normalMag * movementLength, perpindicularDir);
				//printf("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
				targetPosition.add(perpComponent);
			}
		}
		else {
			//printf("movementLength don't normalize a zero vector\n");
		}
	}

	void stepForwardAndStrafe(CollisionWorld collisionWorld, Vector3 walkMove) {
		// printf("m_normalizedDirection=%f,%f,%f\n",
		// 	m_normalizedDirection[0],m_normalizedDirection[1],m_normalizedDirection[2]);
		// phase 2: forward and strafe
		Transform start = Transform();
		Transform end = Transform();
		targetPosition.add2(currentPosition, walkMove);
		start.setIdentity ();
		end.setIdentity ();

		double fraction = 1.0;
		Vector3 distance2Vec = Vector3.zero();
		distance2Vec.sub2(currentPosition, targetPosition);
		double distance2 = distance2Vec.length2;
		//printf("distance2=%f\n",distance2);

		if (touchingContact) {
			if (normalizedDirection.dot(touchingNormal) > 0.0) {
				updateTargetPositionBasedOnCollision(touchingNormal);
			}
		}

		int maxIter = 10;

		while (fraction > 0.01 && maxIter-- > 0) {
			start.origin.setFrom(currentPosition);
			end.origin.setFrom(targetPosition);
      Vector3 sweepDirNegative = Vector3.zero();
      sweepDirNegative.sub2(currentPosition, targetPosition);

			_KinematicClosestNotMeConvexResultCallback callback = _KinematicClosestNotMeConvexResultCallback(ghostObject, sweepDirNegative, -1.0);

			callback.collisionFilterGroup = _getGhostObject().getBroadphaseHandle()?.collisionFilterGroup ?? -1;
			callback.collisionFilterMask = _getGhostObject().getBroadphaseHandle()?.collisionFilterMask ?? 0;

			double margin = convexShape.getMargin();
			convexShape.setMargin(margin + addedMargin);

			if (useGhostObjectSweepTest) {
				ghostObject.convexSweepTest(convexShape, start, end, callback, collisionWorld.getDispatchInfo().allowedCcdPenetration);
			}
			else {
				collisionWorld.convexSweepTest(convexShape, start, end, callback);
			}

			convexShape.setMargin(margin);

			fraction -= callback.closestHitFraction;

			if (callback.hasHit()) {
				// we moved only a fraction
				Vector3 hitDistanceVec = Vector3.zero();
				hitDistanceVec.sub2(callback.hitPointWorld, currentPosition);
//				double hitDistance = hitDistanceVec.length;

				// if the distance is farther than the collision margin, move
				//if (hitDistance > addedMargin) {
				//	//printf("callback.m_closestHitFraction=%f\n",callback.m_closestHitFraction);
				//	currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
				//}

				updateTargetPositionBasedOnCollision(callback.hitNormalWorld);

				Vector3 currentDir = Vector3.zero();
				currentDir.sub2(targetPosition, currentPosition);
				distance2 = currentDir.length2;
				if (distance2 > BulletGlobals.simdEpsilon) {
					currentDir.normalize();
					// see Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners."
					if (currentDir.dot(normalizedDirection) <= 0.0) {
						break;
					}
				}
				else {
					//printf("currentDir: don't normalize a zero vector\n");
					break;
				}
			}
			else {
				// we moved whole way
				currentPosition.setFrom(targetPosition);
			}

			//if (callback.m_closestHitFraction == 0.f)
			//    break;
		}
	}

	void stepDown(CollisionWorld collisionWorld, double dt) {
		Transform start = Transform();
		Transform end = Transform();

		// phase 3: down
    //		double additionalDownStep = (wasOnGround /*&& !onGround()*/) ? stepHeight : 0.0;
    //		Vector3 stepDrop = Vector3.zero();
    //		stepDrop.scale(currentStepOffset + additionalDownStep, upAxisDirection[upAxis]);
    //		double downVelocity = (additionalDownStep == 0.0 && verticalVelocity<0.0?-verticalVelocity:0.0) * dt;
    //		Vector3 gravity_drop = Vector3.zero();
    //		gravity_drop.scale(downVelocity, upAxisDirection[upAxis]);
    //		targetPosition.sub(stepDrop);
    //		targetPosition.sub(gravity_drop);
                
    double downVelocity = (verticalVelocity<0.0?-verticalVelocity:0.0) * dt;
    if(downVelocity > 0.0 && downVelocity < stepHeight&& (wasOnGround || !wasJumping)){
      downVelocity = stepHeight;
    }
    Vector3 stepDrop = Vector3.zero();
    stepDrop.scaleFrom(currentStepOffset + downVelocity, _upAxisDirection[upAxis]);
		targetPosition.sub(stepDrop);

		start.setIdentity ();
		end.setIdentity ();

		start.origin.setFrom(currentPosition);
		end.origin.setFrom(targetPosition);

		_KinematicClosestNotMeConvexResultCallback callback = _KinematicClosestNotMeConvexResultCallback(ghostObject, _upAxisDirection[upAxis], maxSlopeCosine);
		callback.collisionFilterGroup = _getGhostObject().getBroadphaseHandle()?.collisionFilterGroup ?? -1;
		callback.collisionFilterMask = _getGhostObject().getBroadphaseHandle()?.collisionFilterMask ?? 0;

		if (useGhostObjectSweepTest) {
			ghostObject.convexSweepTest(convexShape, start, end, callback, collisionWorld.getDispatchInfo().allowedCcdPenetration);
		}
		else {
			collisionWorld.convexSweepTest(convexShape, start, end, callback);
		}

		if (callback.hasHit()) {
			// we dropped a fraction of the height -> hit floor
			currentPosition.interpolate(currentPosition, targetPosition, callback.closestHitFraction);
			verticalVelocity = 0.0;
			verticalOffset = 0.0;
      wasJumping = false;
		}
		else {
			// we dropped the full height
			currentPosition.setFrom(targetPosition);
		}
	}
}
////////////////////////////////////////////////////////////////////////////

class _KinematicClosestNotMeRayResultCallback extends ClosestRayResultCallback {
  CollisionObject me;

  _KinematicClosestNotMeRayResultCallback(this.me):super(Vector3.zero(), Vector3.zero());

  @override
  double addSingleResult(LocalRayResult rayResult, bool normalInWorldSpace) {
    if (rayResult.collisionObject == me) {
      return 1.0;
    }
    return super.addSingleResult(rayResult, normalInWorldSpace);
  }
}

////////////////////////////////////////////////////////////////////////////

class _KinematicClosestNotMeConvexResultCallback extends ClosestConvexResultCallback {
  CollisionObject me;
  final Vector3 up;
  double minSlopeDot;

  _KinematicClosestNotMeConvexResultCallback(this.me, this.up, this.minSlopeDot):super(Vector3.zero(), Vector3.zero());

  @override
  double addSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace) {
    if (!(convexResult.hitCollisionObject?.hasContactResponse() ?? true)){
      return 1.0;
    }
    if (convexResult.hitCollisionObject == me) {
      return 1.0;
    }
    
    Vector3 hitNormalWorld;
    if (normalInWorldSpace) {
      hitNormalWorld = convexResult.hitNormalLocal;
    } else {
      //need to transform normal into worldspace
      hitNormalWorld = Vector3.zero();
      convexResult.hitCollisionObject?.getWorldTransform(Transform()).basis.transformFrom(convexResult.hitNormalLocal, hitNormalWorld);
    }
    
    double dotUp = up.dot(hitNormalWorld);
    if (dotUp < minSlopeDot) {
      return 1.0;
    }

    return super.addSingleResult(convexResult, normalInWorldSpace);
  }
}