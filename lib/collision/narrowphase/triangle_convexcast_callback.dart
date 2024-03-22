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

//import "package:bullet_physics/collision/narrowphase/gjk_epa_penetration_depth_solver.dart";
import "package:bullet_physics/collision/narrowphase/sub_simplex_convex_cast.dart";
import "package:bullet_physics/collision/narrowphase/voronoi_simplex_solver.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/linearmath/transform.dart";
import 'package:vector_math/vector_math.dart';

import "package:bullet_physics/collision/narrowphase/convex_cast.dart";
import "package:bullet_physics/collision/shapes/triangle_shape.dart";

/**
 *
 * @author jezek2
 */
abstract class TriangleConvexcastCallback extends TriangleCallback {

	ConvexShape convexShape;
	final Transform convexShapeFrom = Transform();
	final Transform convexShapeTo = Transform();
	final Transform triangleToWorld = Transform();
	double hitFraction = 1;
	double triangleCollisionMargin;

	TriangleConvexcastCallback(this.convexShape, Transform convexShapeFrom, Transform convexShapeTo, Transform? triangleToWorld, this.triangleCollisionMargin) {
		this.convexShapeFrom.copy(convexShapeFrom);
		this.convexShapeTo.copy(convexShapeTo);
    if(triangleToWorld != null){
		  this.triangleToWorld.copy(triangleToWorld);
    }
	}
	
  @override
	void processTriangle(List<Vector3> triangle, int partId, int triangleIndex) {
		TriangleShape triangleShape = TriangleShape(triangle[0], triangle[1], triangle[2]);
		triangleShape.setMargin(triangleCollisionMargin);

		VoronoiSimplexSolver simplexSolver = VoronoiSimplexSolver();
		//GjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = GjkEpaPenetrationDepthSolver();

		//#define  USE_SUBSIMPLEX_CONVEX_CAST 1
		//if you reenable USE_SUBSIMPLEX_CONVEX_CAST see commented out code below
		//#ifdef USE_SUBSIMPLEX_CONVEX_CAST
		// TODO: implement ContinuousConvexCollision
		SubSimplexConvexCast convexCaster = SubSimplexConvexCast(convexShape, triangleShape, simplexSolver);
		//#else
		// //btGjkConvexCast	convexCaster(m_convexShape,&triangleShape,&simplexSolver);
		//btContinuousConvexCollision convexCaster(m_convexShape,&triangleShape,&simplexSolver,&gjkEpaPenetrationSolver);
		//#endif //#USE_SUBSIMPLEX_CONVEX_CAST

		CastResult castResult = CastResult();
		castResult.fraction = 1;
		if (convexCaster.calcTimeOfImpact(convexShapeFrom, convexShapeTo, triangleToWorld, triangleToWorld, castResult)) {
			// add hit
			if (castResult.normal.length2 > 0.0001) {
				if (castResult.fraction < hitFraction) {

					/* btContinuousConvexCast's normal is already in world space */
					/*
					//#ifdef USE_SUBSIMPLEX_CONVEX_CAST
					// rotate normal into worldspace
					convexShapeFrom.basis.transform(castResult.normal);
					//#endif //USE_SUBSIMPLEX_CONVEX_CAST
					*/
					castResult.normal.normalize();

					reportHit(castResult.normal,
							castResult.hitPoint,
							castResult.fraction,
							partId,
							triangleIndex);
				}
			}
		}
	}

	double reportHit(Vector3 hitNormalLocal, Vector3 hitPointLocal, double hitFraction, int partId, int triangleIndex);
}
