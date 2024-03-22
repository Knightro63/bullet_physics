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

import 'package:bullet_physics/collision/shapes/collision_shape.dart';
import 'package:bullet_physics/collision/shapes/triangle_callback.dart';
import 'package:vector_math/vector_math.dart';

/**
 * ConcaveShape class provides an interface for non-moving (static) concave shapes.
 * 
 * @author jezek2
 */
abstract class ConcaveShape extends CollisionShape {

	double collisionMargin = 0;

	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax);

  @override
	double getMargin() {
		return collisionMargin;
	}
  @override
	void setMargin(double margin) {
		collisionMargin = margin;
	}
}
