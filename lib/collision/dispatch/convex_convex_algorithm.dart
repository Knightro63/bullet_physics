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

import "package:bullet_physics/collision/broadphase/collision_algorithm.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm_construction_info.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/collision_algorithm_create_func.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/narrowphase/convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/convex_penetration_depth_solver.dart";
import "package:bullet_physics/collision/narrowphase/discrete_collision_detector_interface.dart";
import "package:bullet_physics/collision/narrowphase/gjk_convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/gjk_pair_detector.dart";
import "package:bullet_physics/collision/narrowphase/simplex_solver_interface.dart";
import "package:bullet_physics/collision/narrowphase/voronoi_simplex_solver.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/shapes/sphere_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

class ConvexConvexAlgorithm extends CollisionAlgorithm {
	GjkPairDetector _gjkPairDetector = GjkPairDetector();
	bool ownManifold = false;
	PersistentManifold? manifoldPtr;
	bool lowLevelOfDetail = false;
	
  static CCAFunc CreateFunction(simplexSolver, pdSolver) => CCAFunc(simplexSolver, pdSolver);

	void init(
    PersistentManifold? mf, 
    CollisionAlgorithmConstructionInfo ci, 
    CollisionObject? body0, 
    CollisionObject? body1, 
    SimplexSolverInterface simplexSolver, 
    ConvexPenetrationDepthSolver pdSolver
  ) {
		initCA(ci);
		_gjkPairDetector.init(null, null, simplexSolver, pdSolver);
		manifoldPtr = mf;
	}
	
	@override
	void destroy() {
		if (ownManifold) {
			if (manifoldPtr != null) {
				dispatcher?.releaseManifold(manifoldPtr!);
			}
			manifoldPtr = null;
		}
	}

	void setLowLevelOfDetail(bool useLowLevel) {
		lowLevelOfDetail = useLowLevel;
	}

	/**
	 * Convex-Convex collision algorithm.
	 */
	@override
	void processCollision(CollisionObject? body0, CollisionObject? body1, DispatcherInfo? dispatchInfo, ManifoldResult? resultOut) {
		if (manifoldPtr == null) {
			// swapped?
			manifoldPtr = dispatcher?.getNewManifold(body0, body1);
			ownManifold = true;
		}
		resultOut?.setPersistentManifold(manifoldPtr!);

		ConvexShape? min0 = body0?.getCollisionShape() as ConvexShape?;
		ConvexShape? min1 = body1?.getCollisionShape() as ConvexShape?;
		ClosestPointInput input = ClosestPointInput();

		input.init();
		_gjkPairDetector.setMinkowskiA(min0);
		_gjkPairDetector.setMinkowskiB(min1);
		input.maximumDistanceSquared = (min0?.getMargin() ?? 0) + (min1?.getMargin() ?? 0) + (manifoldPtr?.getContactBreakingThreshold() ?? 0);
		input.maximumDistanceSquared *= input.maximumDistanceSquared;

		body0?.getWorldTransform(input.transformA);
		body1?.getWorldTransform(input.transformB);

		_gjkPairDetector.getClosestPoints(input, resultOut, dispatchInfo?.debugDraw);

		if (ownManifold) {
			resultOut?.refreshContactPoints();
		}
	}

	static bool _disableCcd = false;
	
	@override
	double calculateTimeOfImpact(CollisionObject? col0, CollisionObject? col1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
		Vector3 tmp = Vector3.zero();
		
		Transform tmpTrans1 = Transform();
		Transform tmpTrans2 = Transform();

		// Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold

		// Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
		double resultFraction = 1;

		tmp.sub2(col0?.getInterpolationWorldTransform(tmpTrans1).origin, col0?.getWorldTransform(tmpTrans2).origin);
		double squareMot0 = tmp.length2;

		tmp.sub2(col1?.getInterpolationWorldTransform(tmpTrans1).origin, col1?.getWorldTransform(tmpTrans2).origin);
		double squareMot1 = tmp.length2;

		if (squareMot0 < (col0?.getCcdSquareMotionThreshold() ?? 0) &&
				squareMot1 < (col1?.getCcdSquareMotionThreshold() ?? 0)) {
			return resultFraction;
		}

		if (_disableCcd) {
			return 1;
		}
		
		Transform tmpTrans3 = Transform();
		Transform tmpTrans4 = Transform();

		// An adhoc way of testing the Continuous Collision Detection algorithms
		// One object is approximated as a sphere, to simplify things
		// Starting in penetration should report no time of impact
		// For proper CCD, better accuracy and handling of 'allowed' penetration should be added
		// also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)

		// Convex0 against sphere for Convex1
		{
			ConvexShape? convex0 = col0?.getCollisionShape() as ConvexShape?;

			SphereShape sphere1 = SphereShape(col1?.getCcdSweptSphereRadius() ?? 0); // todo: allow non-zero sphere sizes, for better approximation
			CastResult result = CastResult();
			VoronoiSimplexSolver voronoiSimplex = VoronoiSimplexSolver();
			//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
			///Simplification, one object is simplified as a sphere
			GjkConvexCast ccd1 = GjkConvexCast(convex0, sphere1, voronoiSimplex);
			//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
			if (ccd1.calcTimeOfImpact(
          col0?.getWorldTransform(tmpTrans1), 
          col0?.getInterpolationWorldTransform(tmpTrans2),
					col1?.getWorldTransform(tmpTrans3), 
          col1?.getInterpolationWorldTransform(tmpTrans4), 
          result
        )
      ) {
				if ((col0?.getHitFraction() ?? 0) > result.fraction) {
					col0?.setHitFraction(result.fraction);
				}
				if ((col1?.getHitFraction() ?? 0) > result.fraction) {
					col1?.setHitFraction(result.fraction);
				}
				if (resultFraction > result.fraction) {
					resultFraction = result.fraction;
				}
			}
		}

		// Sphere (for convex0) against Convex1
		{
			ConvexShape? convex1 = col1?.getCollisionShape() as ConvexShape?;

			SphereShape sphere0 = SphereShape(col0?.getCcdSweptSphereRadius() ?? 0); // todo: allow non-zero sphere sizes, for better approximation
			CastResult result = CastResult();
			VoronoiSimplexSolver voronoiSimplex = VoronoiSimplexSolver();
			//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
			///Simplification, one object is simplified as a sphere
			GjkConvexCast ccd1 = GjkConvexCast(sphere0, convex1, voronoiSimplex);
			//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
			if (ccd1.calcTimeOfImpact(
          col0?.getWorldTransform(tmpTrans1), 
          col0?.getInterpolationWorldTransform(tmpTrans2),
					col1?.getWorldTransform(tmpTrans3), 
          col1?.getInterpolationWorldTransform(tmpTrans4), 
          result
        )) {
				//store result.m_fraction in both bodies

				if ((col0?.getHitFraction() ?? 0) > result.fraction) {
					col0?.setHitFraction(result.fraction);
				}

				if ((col1?.getHitFraction() ?? 0) > result.fraction) {
					col1?.setHitFraction(result.fraction);
				}

				if (resultFraction > result.fraction) {
					resultFraction = result.fraction;
				}

			}
		}

		return resultFraction;
	}

	@override
	void getAllContactManifolds(ObjectArrayList<PersistentManifold> manifoldArray) {
		// should we use ownManifold to avoid adding duplicates?
		if (manifoldPtr != null && ownManifold) {
			manifoldArray.add(manifoldPtr!);
		}
	}
	
	PersistentManifold? getManifold() {
		return manifoldPtr;
	}
}


////////////////////////////////////////////////////////////////////////////
class CCAFunc extends CollisionAlgorithmCreateFunc {
  ConvexPenetrationDepthSolver pdSolver;
  SimplexSolverInterface simplexSolver;

  CCAFunc(this.simplexSolver, this.pdSolver);

  @override
  CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
    ConvexConvexAlgorithm algo = ConvexConvexAlgorithm();
    algo.init(ci.manifold, ci, body0, body1, simplexSolver, pdSolver);
    return algo;
  }

  @override
  void releaseCollisionAlgorithm(CollisionAlgorithm algo) {}
}