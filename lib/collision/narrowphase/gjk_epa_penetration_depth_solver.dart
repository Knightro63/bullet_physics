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



import "package:bullet_physics/collision/narrowphase/convex_penetration_depth_solver.dart";
import "package:bullet_physics/collision/narrowphase/gjk_epa_solver.dart";
import "package:bullet_physics/collision/narrowphase/simplex_solver_interface.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/linearmath/i_debug_draw.dart";
import "package:bullet_physics/linearmath/transform.dart";
import 'package:vector_math/vector_math.dart';

/**
 * GjkEpaPenetrationDepthSolver uses the Expanding Polytope Algorithm to calculate
 * the penetration depth between two convex shapes.
 * 
 * @author jezek2
 */
class GjkEpaPenetrationDepthSolver extends ConvexPenetrationDepthSolver {
	GjkEpaSolver _gjkEpaSolver = GjkEpaSolver();

  @override
	bool calcPenDepth(
    SimplexSolverInterface? simplexSolver,
		ConvexShape? pConvexA, 
    ConvexShape? pConvexB,
		Transform transformA, 
    Transform transformB,
		Vector3 v, 
    Vector3 wWitnessOnA, 
    Vector3 wWitnessOnB,
		IDebugDraw? debugDraw/*, btStackAlloc* stackAlloc*/)
	{
		double radialmargin = 0;

		// JAVA NOTE: 2.70b1: update when GjkEpaSolver2 is ported
		
		Results results = Results();
		if (_gjkEpaSolver.collide(
        pConvexA, 
        transformA,
				pConvexB, 
        transformB,
				radialmargin/*,stackAlloc*/, 
        results
      )
    ) {
			//debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
			//resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
			wWitnessOnA.setFrom(results.witnesses[0]);
			wWitnessOnB.setFrom(results.witnesses[1]);
			return true;
		}

		return false;
	}

}
