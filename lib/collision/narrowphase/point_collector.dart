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

import 'package:bullet_physics/collision/narrowphase/discrete_collision_detector_interface.dart';
import 'package:vector_math/vector_math.dart';

class PointCollector extends Result{

	final Vector3 normalOnBInWorld = Vector3.zero();
	final Vector3 pointInWorld = Vector3.zero();
	double distance = 1e30; // negative means penetration

	bool hasResult = false;
	
  @override
	void setShapeIdentifiers(int partId0, int index0, int partId1, int index1) {}
  @override
	void addContactPoint(Vector3 normalOnBInWorld, Vector3 pointInWorld, double depth) {
		if (depth < distance) {
			hasResult = true;
			this.normalOnBInWorld.setFrom(normalOnBInWorld);
			this.pointInWorld.setFrom(pointInWorld);
			// negative means penetration
			distance = depth;
		}
	}

}
