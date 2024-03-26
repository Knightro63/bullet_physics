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

import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";

class CompoundShapeChild {
	final Transform transform = Transform();
	CollisionShape? childShape;
	BroadphaseNativeType? childShapeType;
	double childMargin = 0;

	bool equals(Object? obj) {
		if (obj == null || (obj is! CompoundShapeChild)) return false;
		CompoundShapeChild child = obj;
		return transform.equals(child.transform) &&
		       childShape == child.childShape &&
		       childShapeType == child.childShapeType &&
		       childMargin == child.childMargin;
	}

	@override
	int get hashCode => _gethashCode();
  int _gethashCode(){
		int hash = 7;
		hash = 19 * hash + transform.hashCode;
		hash = 19 * hash + childShape.hashCode;
		hash = 19 * hash + childShapeType.hashCode;
		hash = 19 * hash + childMargin.toInt();
		return hash;
	}

  /// Check if two matrices are the same.
  @override
  bool operator ==(Object other) =>
      (other is CompoundShapeChild) &&
      (childShape == other.childShape) &&
      (childShapeType == other.childShapeType) &&
      (childMargin == other.childMargin);

}
