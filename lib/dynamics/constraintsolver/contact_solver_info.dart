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

import 'package:bullet_physics/dynamics/constraintsolver/solver_mode.dart';

/**
 * Current state of contact solver.
 * 
 * @author jezek2
 */
class ContactSolverInfo {
	double tau = 0.6;
	double damping = 1;
	double friction = 0.3;
	double timeStep = 0;
	double restitution = 0;
	int numIterations = 10;
	double maxErrorReduction = 20;
	double sor = 1.3;
	double erp = 0.2; // used as Baumgarte factor
	double erp2 = 0.1; // used in Split Impulse
	bool splitImpulse = false;
	double splitImpulsePenetrationThreshold = -0.02;
	double linearSlop = 0;
	double warmstartingFactor = 0.85;
	
	int solverMode = SolverMode.randomOrder | SolverMode.cacheFriendly | SolverMode.useWarmStarting;
	
	ContactSolverInfo([ContactSolverInfo? g]) {
    if(g != null){
      tau = g.tau;
      damping = g.damping;
      friction = g.friction;
      timeStep = g.timeStep;
      restitution = g.restitution;
      numIterations = g.numIterations;
      maxErrorReduction = g.maxErrorReduction;
      sor = g.sor;
      erp = g.erp;
    }
	}
	
}