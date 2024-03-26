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
import "package:bullet_physics/collision/narrowphase/simplex_solver_interface.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class SubSimplexConvexCast extends ConvexCast {
	//final BulletStack stack = BulletStack.get();
	
	static const int _maxIterations = 32;
	late SimplexSolverInterface _simplexSolver;
	late ConvexShape _convexA;
	late ConvexShape _convexB;

	SubSimplexConvexCast(ConvexShape shapeA, ConvexShape shapeB, SimplexSolverInterface simplexSolver) {
		_convexA = shapeA;
		_convexB = shapeB;
		_simplexSolver = simplexSolver;
	}
  @override
	bool calcTimeOfImpact(Transform fromA, Transform toA, Transform? fromB, Transform? toB, CastResult result) {
		Vector3 tmp = Vector3.zero();
		
		_simplexSolver.reset();

		Vector3 linVelA = Vector3.zero();
		Vector3 linVelB = Vector3.zero();
		linVelA.sub2(toA.origin, fromA.origin);
		linVelB.sub2(toB?.origin, fromB?.origin);
		
		double lambda = 0;
		
		Transform interpolatedTransA = Transform.formTransfrom(fromA);
		Transform interpolatedTransB = Transform.formTransfrom(fromB);

		// take relative motion
		Vector3 r = Vector3.zero();
		r.sub2(linVelA, linVelB);
		
		Vector3 v = Vector3.zero();

		tmp.negateFrom(r);
		MatrixUtil.transposeTransform(tmp, tmp, fromA.basis);
		Vector3 supVertexA = _convexA.localGetSupportingVertex(tmp, Vector3.zero());
		fromA.transform(supVertexA);
    if(fromB?.basis != null){
		  MatrixUtil.transposeTransform(tmp, r, fromB!.basis);
    }
		Vector3 supVertexB = _convexB.localGetSupportingVertex(tmp, Vector3.zero());
		fromB?.transform(supVertexB);
		
		v.sub2(supVertexA, supVertexB);
		
		int maxIter = _maxIterations;

		Vector3 n = Vector3.zero();
		n.setValues(0, 0, 0);
		//bool hasResult = false;
		//Vector3 c = Vector3.zero();
		//double lastLambda = lambda;

		double dist2 = v.length2;
		double epsilon = 0.0001;
		Vector3 w = Vector3.zero();
    //p = Vector3.zero();
		double vDotR;

		while ((dist2 > epsilon) && (maxIter--) != 0) {
			tmp.negateFrom(v);
			MatrixUtil.transposeTransform(tmp, tmp, interpolatedTransA.basis);
			_convexA.localGetSupportingVertex(tmp, supVertexA);
			interpolatedTransA.transform(supVertexA);
			
			MatrixUtil.transposeTransform(tmp, v, interpolatedTransB.basis);
			_convexB.localGetSupportingVertex(tmp, supVertexB);
			interpolatedTransB.transform(supVertexB);
			
			w.sub2(supVertexA, supVertexB);

			double vDotW = v.dot(w);

			if (lambda > 1) {
				return false;
			}
			
			if (vDotW > 0) {
				vDotR = v.dot(r);

				if (vDotR >= -(BulletGlobals.fltEpsilon * BulletGlobals.fltEpsilon)) {
					return false;
				}
				else {
					lambda = lambda - vDotW / vDotR;
					VectorUtil.setInterpolate3(interpolatedTransA.origin, fromA.origin, toA.origin, lambda);
          if(fromB?.origin !=null && toB?.origin != null){
					  VectorUtil.setInterpolate3(interpolatedTransB.origin, fromB!.origin, toB!.origin, lambda);
          }
					w.sub2(supVertexA, supVertexB);
					n.setFrom(v);
				}
			}
			_simplexSolver.addVertex(w, supVertexA , supVertexB);
			if (_simplexSolver.closest(v)) {
				dist2 = v.length2;
			}
			else {
				dist2 = 0;
			}
		}
    		
		result.fraction = lambda;
		if (n.length2 >= BulletGlobals.simdEpsilon * BulletGlobals.simdEpsilon) {
			result.normal.normalizeFrom(n);
		}
		else {
			result.normal.setValues(0, 0, 0);
		}

		// don't report time of impact for motion away from the contact normal (or causes minor penetration)
		if (result.normal.dot(r) >= -result.allowedPenetration){
			return false;
    }

		Vector3 hitA = Vector3.zero();
		Vector3 hitB = Vector3.zero();
		_simplexSolver.computePoints(hitA,hitB);
		result.hitPoint.setFrom(hitB);
		return true;
	}

}
