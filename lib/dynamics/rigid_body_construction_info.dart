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

import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/linearmath/motion_state.dart";
import "package:bullet_physics/linearmath/transform.dart";
import 'package:vector_math/vector_math.dart';

class RigidBodyConstructionInfo {
	double mass;

	/**
	 * When a motionState is provided, the rigid body will initialize its world transform
	 * from the motion state. In this case, startWorldTransform is ignored.
	 */
	MotionState? motionState;
	final Transform startWorldTransform = Transform();

	CollisionShape? collisionShape;
	final Vector3 localInertia = Vector3.zero();
	double linearDamping = 0;
	double angularDamping = 0;

	/** Best simulation results when friction is non-zero. */
	double friction = 0.5;
	/** Best simulation results using zero restitution. */
	double restitution = 0;

	double linearSleepingThreshold = 0.8;
	double angularSleepingThreshold = 1.0;

	/**
	 * Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
	 * Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics
	 * system has improved, this should become obsolete.
	 */
	bool additionalDamping = false;
	double additionalDampingFactor = 0.005;
	double additionalLinearDampingThresholdSqr = 0.01;
	double additionalAngularDampingThresholdSqr = 0.01;
	double additionalAngularDampingFactor = 0.01;
	
	RigidBodyConstructionInfo(this.mass,this.motionState,this.collisionShape, [Vector3? localInertia]) {
    localInertia ??= Vector3(0, 0, 0);
		this.localInertia.setFrom(localInertia);
		startWorldTransform.setIdentity();
	}
}
