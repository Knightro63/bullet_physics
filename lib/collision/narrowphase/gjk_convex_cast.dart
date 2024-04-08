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

import "package:bullet_physics/collision/narrowphase/convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/discrete_collision_detector_interface.dart";
import "package:bullet_physics/collision/narrowphase/gjk_pair_detector.dart";
import "package:bullet_physics/collision/narrowphase/point_collector.dart";
import "package:bullet_physics/collision/narrowphase/simplex_solver_interface.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class GjkConvexCast extends ConvexCast {
	static const int _maxIterations = 32;
	
	late SimplexSolverInterface _simplexSolver;
	ConvexShape? _convexA;
	ConvexShape? _convexB;
	GjkPairDetector _gjk = GjkPairDetector();

	GjkConvexCast(ConvexShape? convexA, ConvexShape? convexB, SimplexSolverInterface simplexSolver) {
		_simplexSolver = simplexSolver;
		_convexA = convexA;
		_convexB = convexB;
	}
  
  @override
	bool calcTimeOfImpact(Transform? fromA, Transform? toA, Transform? fromB, Transform? toB, CastResult result) {
		_simplexSolver.reset();

		// compute linear velocity for this interval, to interpolate
		// assume no rotation/angular velocity, assert here?
		Vector3 linVelA = Vector3.zero();
		Vector3 linVelB = Vector3.zero();

		linVelA.sub2(toA?.origin, fromA?.origin);
		linVelB.sub2(toB?.origin, fromB?.origin);

		double radius = 0.001;
		double lambda = 0;
		Vector3 v = Vector3.zero();
		v.setValues(1, 0, 0);

		int maxIter = _maxIterations;

		Vector3 n = Vector3.zero();
		bool hasResult = false;
		Vector3 c = Vector3.zero();
		Vector3 r = Vector3.zero();
		r.sub2(linVelA, linVelB);

		double lastLambda = lambda;
		//btScalar epsilon = btScalar(0.001);

		int numIter = 0;
		// first solution, using GJK

		Transform identityTrans = Transform();
		identityTrans.setIdentity();

		//result.drawCoordSystem(sphereTr);

		PointCollector pointCollector = PointCollector();

		_gjk.init(_convexA, _convexB, _simplexSolver, null); // penetrationDepthSolver);		
		ClosestPointInput input = ClosestPointInput();
		try {
			// we don't use margins during CCD
			//	gjk.setIgnoreMargin(true);
      if(fromA != null){
			  input.transformA.copy(fromA);
      }
      if(fromB != null){
			  input.transformB.copy(fromB);
      }
			_gjk.getClosestPoints(input, pointCollector, null);

			hasResult = pointCollector.hasResult;
			c.setFrom(pointCollector.pointInWorld);

			if (hasResult) {
				double dist;
				dist = pointCollector.distance;
				n.setFrom(pointCollector.normalOnBInWorld);

				// not close enough
				while (dist > radius) {
					numIter++;
					if (numIter > maxIter) {
						return false; // todo: report a failure
					}
					double dLambda = 0;

					double projectedLinearVelocity = r.dot(n);

					dLambda = dist / (projectedLinearVelocity);

					lambda = lambda - dLambda;

					if (lambda > 1) {
						return false;
					}
					if (lambda < 0) {
						return false;					// todo: next check with relative epsilon
					}
					
					if (lambda <= lastLambda) {
						return false;
					//n.setValue(0,0,0);
					//break;
					}
					lastLambda = lambda;

					// interpolate to next lambda
					result.debugDraw(lambda);
					VectorUtil.setInterpolate3(input.transformA.origin, fromA?.origin ?? Vector3.zero(), toA?.origin ?? Vector3.zero(), lambda);
					VectorUtil.setInterpolate3(input.transformB.origin, fromB?.origin ?? Vector3.zero(), toB?.origin ?? Vector3.zero(), lambda);

					_gjk.getClosestPoints(input, pointCollector, null);
					if (pointCollector.hasResult) {
						if (pointCollector.distance < 0) {
							result.fraction = lastLambda;
							n.setFrom(pointCollector.normalOnBInWorld);
							result.normal.setFrom(n);
							result.hitPoint.setFrom(pointCollector.pointInWorld);
							return true;
						}
						c.setFrom(pointCollector.pointInWorld);
						n.setFrom(pointCollector.normalOnBInWorld);
						dist = pointCollector.distance;
					}
					else {
						// ??
						return false;
					}

				}

				// is n normalized?
				// don't report time of impact for motion away from the contact normal (or causes minor penetration)
				if (n.dot(r) >= -result.allowedPenetration) {
					return false;
				}
				result.fraction = lambda;
				result.normal.setFrom(n);
				result.hitPoint.setFrom(c);
				return true;
			}

			return false;
		}
		finally {
		}
	}
	
}
