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

import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/dynamics/constraintsolver/contact_solver_info.dart";
import "package:bullet_physics/dynamics/constraintsolver/typed_constraint.dart";
import "package:bullet_physics/linearmath/i_debug_draw.dart";
import "package:bullet_physics/utils/object_array_list.dart";

abstract class ConstraintSolver {
	//final BulletStack stack = BulletStack.get();
	void prepareSolve (int numBodies, int numManifolds) {}
	/**
	 * Solve a group of constraints.
	 */
	double solveGroup(ObjectArrayList<CollisionObject>? bodies, int numBodies, ObjectArrayList<PersistentManifold>? manifold, int manifoldOffset, int numManifolds, ObjectArrayList<TypedConstraint>? constraints, int constraintsOffset, int numConstraints, ContactSolverInfo? info, IDebugDraw? debugDrawer/*, btStackAlloc* stackAlloc*/, Dispatcher? dispatcher);
	void allSolved(ContactSolverInfo info, IDebugDraw? debugDrawer) {}
	/**
	 * Clear internal cached data and reset random seed.
	 */
	void reset();
}
