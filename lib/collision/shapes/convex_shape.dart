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
import 'package:bullet_physics/linearmath/transform.dart';
import 'package:vector_math/vector_math.dart';

/**
 * ConvexShape is an abstract shape class. It describes general convex shapes
 * using the {@link #localGetSupportingVertex localGetSupportingVertex} interface
 * used in combination with GJK or ConvexCast.
 * 
 * @author jezek2
 */
abstract class ConvexShape extends CollisionShape {

	static const int maxPreferredPenetrationDirections = 10;
	
	Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out);

	//#ifndef __SPU__
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out);

	//notice that the vectors should be unit length
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors);
	//#endif
	
	void getAabbSlow(Transform t, Vector3 aabbMin, Vector3 aabbMax);

	void setLocalScaling(Vector3 scaling);

	Vector3 getLocalScaling(Vector3 out);

	void setMargin(double margin);

	double getMargin();

	int getNumPreferredPenetrationDirections();

	void getPreferredPenetrationDirection(int index, Vector3 penetrationVector);
	
}
