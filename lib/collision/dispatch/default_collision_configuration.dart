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

import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/dispatch/collision_algorithm_create_func.dart";
import "package:bullet_physics/collision/dispatch/collision_configuration.dart";
import "package:bullet_physics/collision/dispatch/compound_collision_algorithm.dart";
import "package:bullet_physics/collision/dispatch/convex_concave_collision_algorithm.dart";
import "package:bullet_physics/collision/dispatch/convex_convex_algorithm.dart";
import "package:bullet_physics/collision/dispatch/convex_plane_collision_algorithm.dart";
import "package:bullet_physics/collision/dispatch/empty_algorithm.dart";
import "package:bullet_physics/collision/dispatch/sphere_sphere_collision_algorithm.dart";
import "package:bullet_physics/collision/narrowphase/convex_penetration_depth_solver.dart";
import "package:bullet_physics/collision/narrowphase/gjk_epa_penetration_depth_solver.dart";
import "package:bullet_physics/collision/narrowphase/voronoi_simplex_solver.dart";

class DefaultCollisionConfiguration extends CollisionConfiguration {

	//default simplex/penetration depth solvers
	VoronoiSimplexSolver simplexSolver = VoronoiSimplexSolver();
	ConvexPenetrationDepthSolver pdSolver = GjkEpaPenetrationDepthSolver();
	
	//default CreationFunctions, filling the m_doubleDispatch table
	late CollisionAlgorithmCreateFunc convexConvexCreateFunc;
	late CollisionAlgorithmCreateFunc convexConcaveCreateFunc;
	late CollisionAlgorithmCreateFunc swappedConvexConcaveCreateFunc;
	late CollisionAlgorithmCreateFunc compoundCreateFunc;
	late CollisionAlgorithmCreateFunc swappedCompoundCreateFunc;
	late CollisionAlgorithmCreateFunc emptyCreateFunc;
	late CollisionAlgorithmCreateFunc sphereSphereCF;
	late CollisionAlgorithmCreateFunc sphereBoxCF;
	late CollisionAlgorithmCreateFunc boxSphereCF;
	late CollisionAlgorithmCreateFunc boxBoxCF;
	late CollisionAlgorithmCreateFunc sphereTriangleCF;
	late CollisionAlgorithmCreateFunc triangleSphereCF;
	late CollisionAlgorithmCreateFunc planeConvexCF;
	late CollisionAlgorithmCreateFunc convexPlaneCF;
	
	DefaultCollisionConfiguration() {
		convexConvexCreateFunc = ConvexConvexAlgorithm.CreateFunction(simplexSolver, pdSolver);
		convexConcaveCreateFunc = ConvexConcaveCollisionAlgorithm.CreateFunction();
		swappedConvexConcaveCreateFunc = ConvexConcaveCollisionAlgorithm.SwappedCreateFunction();
		compoundCreateFunc = CompoundCollisionAlgorithm.CreateFunction();
		swappedCompoundCreateFunc = CompoundCollisionAlgorithm.SwappedCreateFunction();
		emptyCreateFunc = EmptyAlgorithm.CreateFunction();
		sphereSphereCF = SphereSphereCollisionAlgorithm.CreateFunction();

		// convex versus plane
		convexPlaneCF = ConvexPlaneCollisionAlgorithm.CreateFunction();
		planeConvexCF = ConvexPlaneCollisionAlgorithm.CreateFunction();
		planeConvexCF.swapped = true;
	}
	
	@override
	CollisionAlgorithmCreateFunc getCollisionAlgorithmCreateFunc(BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1) {
		if ((proxyType0 == BroadphaseNativeType.sphereShapeProxytype) && (proxyType1 == BroadphaseNativeType.sphereShapeProxytype)) {
			return sphereSphereCF;
		}

		if (proxyType0.isConvex() && (proxyType1 == BroadphaseNativeType.staticPlaneProxytype))
		{
			return convexPlaneCF;
		}

		if (proxyType1.isConvex() && (proxyType0 == BroadphaseNativeType.staticPlaneProxytype))
		{
			return planeConvexCF;
		}

		if (proxyType0.isConvex() && proxyType1.isConvex()) {
			return convexConvexCreateFunc;
		}

		if (proxyType0.isConvex() && proxyType1.isConcave()) {
			return convexConcaveCreateFunc;
		}

		if (proxyType1.isConvex() && proxyType0.isConcave()) {
			return swappedConvexConcaveCreateFunc;
		}

		if (proxyType0.isCompound()) {
			return compoundCreateFunc;
		}
		else {
			if (proxyType1.isCompound()) {
				return swappedCompoundCreateFunc;
			}
		}

		// failed to find an algorithm
		return emptyCreateFunc;
	}

}
