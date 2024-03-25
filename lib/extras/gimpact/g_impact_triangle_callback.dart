/*
 * Dart port of Bullet (c) 2024 @Knightro
 *
 * This source file is part of GIMPACT Library.
 *
 * For the latest info, see http://gimpact.sourceforge.net/
 *
 * Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
 * email: projectileman@yahoo.com
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

import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/extras/gimpact/g_impact_collision_algorithm.dart";
import "package:bullet_physics/extras/gimpact/g_impact_shape_interface.dart";
import "package:bullet_physics/extras/gimpact/triangle_shape_ex.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class GImpactTriangleCallback extends TriangleCallback {
	GImpactCollisionAlgorithm? algorithm;
	CollisionObject? body0;
	CollisionObject? body1;
	GImpactShapeInterface? gimpactshape0;
	bool swapped = false;
	double margin = 0;
	
  @override
	void processTriangle(List<Vector3> triangle, int partId, int triangleIndex) {
		TriangleShapeEx tri1 = TriangleShapeEx(triangle[0], triangle[1], triangle[2]);
		tri1.setMargin(margin);
		if (swapped) {
			algorithm?.setPart0(partId);
			algorithm?.setFace0(triangleIndex);
		}
		else {
			algorithm?.setPart1(partId);
			algorithm?.setFace1(triangleIndex);
		}
		algorithm?.gimpactVSshape(body0, body1, gimpactshape0, tri1, swapped);
	}

}
