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

import "package:bullet_physics/collision/narrowphase/convex_penetration_depth_solver.dart";
import "package:bullet_physics/collision/narrowphase/discrete_collision_detector_interface.dart";
import "package:bullet_physics/collision/narrowphase/simplex_solver_interface.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/core/bullet_stats.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/linearmath/i_debug_draw.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

class GjkPairDetector extends DiscreteCollisionDetectorInterface {
	//final BulletStack stack = BulletStack.get();
	
	// must be above the machine epsilon
	static const double _relError2 = 1.0e-6;
	
	final Vector3 _cachedSeparatingAxis = Vector3(0,0,1);
	ConvexPenetrationDepthSolver? _penetrationDepthSolver;
	SimplexSolverInterface? _simplexSolver;
	ConvexShape? _minkowskiA;
	ConvexShape? _minkowskiB;
	bool _ignoreMargin = false;
	
	// some debugging to fix degeneracy problems
	int lastUsedMethod = -1;
	int curIter = 0;
	int degenerateSimplex = 0;
	int catchDegeneracies = 1;
	
	void init(ConvexShape? objectA, ConvexShape? objectB, SimplexSolverInterface simplexSolver, ConvexPenetrationDepthSolver? penetrationDepthSolver) {		
		_penetrationDepthSolver = penetrationDepthSolver;
		_simplexSolver = simplexSolver;
		_minkowskiA = objectA;
		_minkowskiB = objectB;
	}

  @override
	void getClosestPoints(ClosestPointInput input, Result? output, [IDebugDraw? debugDraw, bool swapResults = false]){
		Vector3 tmp = Vector3.zero();

		double distance = 0;
		Vector3 normalInB = Vector3.zero();
		Vector3 pointOnA = Vector3.zero();
    Vector3 pointOnB = Vector3.zero();
		Transform localTransA = Transform.formTransfrom(input.transformA);
		Transform localTransB = Transform.formTransfrom(input.transformB);
		Vector3 positionOffset = Vector3.zero();
		positionOffset.add2(localTransA.origin, localTransB.origin);
		positionOffset.scale(0.5);
		localTransA.origin.sub(positionOffset);
		localTransB.origin.sub(positionOffset);

		double marginA = _minkowskiA?.getMargin() ?? 0;
		double marginB = _minkowskiB?.getMargin() ?? 0;
		BulletStats.gNumGjkChecks++;

		// for CCD we don't use margins
		if (_ignoreMargin) {
			marginA = 0;
			marginB = 0;
		}

		curIter = 0;
		int gGjkMaxIter = 1000; // this is to catch invalid input, perhaps check for #NaN?
		_cachedSeparatingAxis.setValues(0, 1, 0);

		bool isValid = false;
		bool checkSimplex = false;
		bool checkPenetration = true;
		degenerateSimplex = 0;

		lastUsedMethod = -1;

		{
			double squaredDistance = BulletGlobals.simdInfinity;
			double delta = 0;

			double margin = marginA + marginB;

			_simplexSolver?.reset();

			Vector3 seperatingAxisInA = Vector3.zero();
			Vector3 seperatingAxisInB = Vector3.zero();
			
			Vector3 pInA = Vector3.zero();
			Vector3 qInB = Vector3.zero();
			
			Vector3 pWorld = Vector3.zero();
			Vector3 qWorld = Vector3.zero();
			Vector3 w = Vector3.zero();
			
			Vector3 tmpPointOnA = Vector3.zero();
      Vector3 tmpPointOnB = Vector3.zero();
			Vector3 tmpNormalInB = Vector3.zero();
			
			for (;;){
				seperatingAxisInA.negateFrom(_cachedSeparatingAxis);
				MatrixUtil.transposeTransform(seperatingAxisInA, seperatingAxisInA, input.transformA.basis);

				seperatingAxisInB.setFrom(_cachedSeparatingAxis);
				MatrixUtil.transposeTransform(seperatingAxisInB, seperatingAxisInB, input.transformB.basis);

				_minkowskiA?.localGetSupportingVertexWithoutMargin(seperatingAxisInA, pInA);
				_minkowskiB?.localGetSupportingVertexWithoutMargin(seperatingAxisInB, qInB);

				pWorld.setFrom(pInA);
				localTransA.transform(pWorld);
				
				qWorld.setFrom(qInB);
				localTransB.transform(qWorld);

				w.sub2(pWorld, qWorld);

				delta = _cachedSeparatingAxis.dot(w);

				// potential exit, they don't overlap
				if ((delta > 0) && (delta * delta > squaredDistance * input.maximumDistanceSquared)) {
					checkPenetration = false;
					break;
				}

				// exit 0: the point is already in the simplex, or we didn't come any closer
				if (_simplexSolver!.inSimplex(w)) {
					degenerateSimplex = 1;
					checkSimplex = true;
					break;
				}
				// are we getting any closer ?
				double f0 = squaredDistance - delta;
				double f1 = squaredDistance * _relError2;

				if (f0 <= f1) {
					if (f0 <= 0) {
						degenerateSimplex = 2;
					}
					checkSimplex = true;
					break;
				}
				// add current vertex to simplex
				_simplexSolver?.addVertex(w, pWorld, qWorld);

				// calculate the closest point to the origin (update vector v)
				if (!_simplexSolver!.closest(_cachedSeparatingAxis)) {
					degenerateSimplex = 3;
					checkSimplex = true;
					break;
				}

				if (_cachedSeparatingAxis.length2 < _relError2) {
					degenerateSimplex = 6;
					checkSimplex = true;
					break;
				}
				
				double previousSquaredDistance = squaredDistance;
				squaredDistance = _cachedSeparatingAxis.length2;

				// redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

				// are we getting any closer ?
				if (previousSquaredDistance - squaredDistance <= BulletGlobals.fltEpsilon * previousSquaredDistance) {
					_simplexSolver?.backupClosest(_cachedSeparatingAxis);
					checkSimplex = true;
					break;
				}

				// degeneracy, this is typically due to invalid/uninitialized worldtransforms for a CollisionObject   
				if (curIter++ > gGjkMaxIter) {
					//#if defined(DEBUG) || defined (_DEBUG)   
					if (BulletGlobals.debug) {
						print("btGjkPairDetector maxIter exceeded:%i\n $curIter");
						print("sepAxis=(%f,%f,%f), squaredDistance = %f, shapeTypeA=%i,shapeTypeB=%i\n"+
								[_cachedSeparatingAxis.x,
								_cachedSeparatingAxis.y,
								_cachedSeparatingAxis.z,
								squaredDistance,
								_minkowskiA?.getShapeType(),
								_minkowskiB?.getShapeType()].toString());
					}
					//#endif   
					break;

				}

				bool check = !_simplexSolver!.fullSimplex();
				//bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > SIMD_EPSILON * m_simplexSolver->maxVertex());

				if (!check) {
					// do we need this backup_closest here ?
					_simplexSolver?.backupClosest(_cachedSeparatingAxis);
					break;
				}
			}

			if (checkSimplex) {
				_simplexSolver?.computePoints(pointOnA, pointOnB);
				normalInB.sub2(pointOnA, pointOnB);
				double lenSqr = _cachedSeparatingAxis.length2;
				// valid normal
				if (lenSqr < 0.0001) {
					degenerateSimplex = 5;
				}
				if (lenSqr > BulletGlobals.fltEpsilon * BulletGlobals.fltEpsilon) {
					double rlen = 1 / sqrt(lenSqr);
					normalInB.scale(rlen); // normalize
					double s = sqrt(squaredDistance);

					assert (s > 0);

					tmp.scaleFrom((marginA / s), _cachedSeparatingAxis);
					pointOnA.sub(tmp);

					tmp.scaleFrom((marginB / s), _cachedSeparatingAxis);
					pointOnB.add(tmp);

					distance = ((1 / rlen) - margin);
					isValid = true;

					lastUsedMethod = 1;
				}
				else {
					lastUsedMethod = 2;
				}
			}

			bool catchDegeneratePenetrationCase =
					(catchDegeneracies != 0 && _penetrationDepthSolver != null && degenerateSimplex != 0 && ((distance + margin) < 0.01));

			//if (checkPenetration && !isValid)
			if (checkPenetration && (!isValid || catchDegeneratePenetrationCase)) {
				// penetration case

				// if there is no way to handle penetrations, bail out
				if (_penetrationDepthSolver != null) {
					// Penetration depth case.
					BulletStats.gNumDeepPenetrationChecks++;

					bool isValid2 = _penetrationDepthSolver?.calcPenDepth(
            _simplexSolver,
            _minkowskiA, 
            _minkowskiB,
            localTransA, 
            localTransB,
            _cachedSeparatingAxis, 
            tmpPointOnA, 
            tmpPointOnB,
            debugDraw
          ) ?? true;

					if (isValid2) {
						tmpNormalInB.sub2(tmpPointOnB, tmpPointOnA);

						double lenSqr = tmpNormalInB.length2;
						if (lenSqr > (BulletGlobals.fltEpsilon * BulletGlobals.fltEpsilon)) {
							tmpNormalInB.scale(1 / sqrt(lenSqr));
							tmp.sub2(tmpPointOnA, tmpPointOnB);
							double distance2 = -tmp.length;
							// only replace valid penetrations when the result is deeper (check)
							if (!isValid || (distance2 < distance)) {
								distance = distance2;
								pointOnA.setFrom(tmpPointOnA);
								pointOnB.setFrom(tmpPointOnB);
								normalInB.setFrom(tmpNormalInB);
								isValid = true;
								lastUsedMethod = 3;
							}
							else {

							}
						}
						else {
							//isValid = false;
							lastUsedMethod = 4;
						}
					}
					else {
						lastUsedMethod = 5;
					}

				}
			}
		}
		if (isValid) {
			tmp.add2(pointOnB, positionOffset);
			output?.addContactPoint(
        normalInB,
        tmp,
        distance
      );
		}
	}

	void setMinkowskiA(ConvexShape? minkA) {
		_minkowskiA = minkA;
	}

	void setMinkowskiB(ConvexShape? minkB) {
		_minkowskiB = minkB;
	}

	void setCachedSeperatingAxis(Vector3 seperatingAxis) {
		_cachedSeparatingAxis.setFrom(seperatingAxis);
	}

	void setPenetrationDepthSolver(ConvexPenetrationDepthSolver penetrationDepthSolver) {
		_penetrationDepthSolver = penetrationDepthSolver;
	}

	/**
	 * Don't use setIgnoreMargin, it's for Bullet's internal use.
	 */
	void setIgnoreMargin(bool ignoreMargin) {
		_ignoreMargin = ignoreMargin;
	}
}
