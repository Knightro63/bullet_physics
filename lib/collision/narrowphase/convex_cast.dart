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

import "package:bullet_physics/linearmath/i_debug_draw.dart";
import "package:bullet_physics/linearmath/transform.dart";
import 'package:vector_math/vector_math.dart';

/**
 * ConvexCast is an interface for casting.
 * 
 * @author jezek2
 */
abstract class ConvexCast {
  
	bool calcTimeOfImpact(Transform fromA, Transform toA, Transform? fromB, Transform? toB, CastResult result);
}

class CastResult {
  CastResult();

  final Transform hitTransformA = Transform();
  final Transform hitTransformB = Transform();
  
  final Vector3 normal = Vector3.zero();
  final Vector3 hitPoint = Vector3.zero();
  double fraction = 1e30; // input and output
  double allowedPenetration = 0;
  
  IDebugDraw? debugDrawer;
  
  void debugDraw(double fraction) {}
  void	drawCoordSystem(Transform trans) {}
}