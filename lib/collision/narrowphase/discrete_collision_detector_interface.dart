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

abstract class DiscreteCollisionDetectorInterface {
	void getClosestPoints(ClosestPointInput input,Result output, IDebugDraw debugDraw, [bool swapResults = false]);
}

abstract class Result {
  void setShapeIdentifiers(int partId0, int index0, int partId1, int index1);
  void addContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, double depth);
}

class ClosestPointInput {
  final Transform transformA = Transform();
  final Transform transformB = Transform();
  double maximumDistanceSquared = double.infinity;

  ClosestPointInput() {
    init();
  }
  void init() {
    maximumDistanceSquared = double.infinity;
  }
}