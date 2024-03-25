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

import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * ConvexInternalShape is an internal base class, shared by most convex shape implementations.
 * 
 * @author jezek2
 */
abstract class ConvexInternalShape extends ConvexShape {

	// local scaling. collisionMargin is not scaled !
	final Vector3 localScaling = Vector3(1, 1, 1);
	final Vector3 implicitShapeDimensions = Vector3.zero();
	double collisionMargin = BulletGlobals.convexDistanceMargin;

	/**
	 * getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version.
	 */
	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		getAabbSlow(t, aabbMin, aabbMax);
	}
	
	@override
	void getAabbSlow(Transform trans, Vector3 minAabb, Vector3 maxAabb) {
		double margin = getMargin();
		Vector3 vec = Vector3.zero();
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();
		
		for (int i=0;i<3;i++)
		{
			vec.setValues(0, 0, 0);
			VectorUtil.setCoord(vec, i, 1);

			MatrixUtil.transposeTransform(tmp1, vec, trans.basis);
			localGetSupportingVertex(tmp1, tmp2);
			
			trans.transform(tmp2);

			VectorUtil.setCoord(maxAabb, i, VectorUtil.getCoord(tmp2, i) + margin);

			VectorUtil.setCoord(vec, i, -1);

			MatrixUtil.transposeTransform(tmp1, vec, trans.basis);
			localGetSupportingVertex(tmp1, tmp2);
			trans.transform(tmp2);

			VectorUtil.setCoord(minAabb, i, VectorUtil.getCoord(tmp2, i) - margin);
		}
	}

	@override
	Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
		Vector3 supVertex = localGetSupportingVertexWithoutMargin(vec, out);

		if (getMargin() != 0) {
			Vector3 vecnorm = Vector3.copy(vec);
			if (vecnorm.length2 < (BulletGlobals.fltEpsilon * BulletGlobals.fltEpsilon)) {
				vecnorm.setValues(-1, -1, -1);
			}
			vecnorm.normalize();
			supVertex.scaleAdd(getMargin(), vecnorm, supVertex);
		}
		return out;
	}
	
  @override
	void setLocalScaling(Vector3 scaling) {
		localScaling.absoluteFrom(scaling);
	}
	@override
	Vector3 getLocalScaling(Vector3 out) {
		out.setFrom(localScaling);
		return out;
	}
  @override
	double getMargin() {
		return collisionMargin;
	}
  @override
	void setMargin(double margin) {
		collisionMargin = margin;
	}

	@override
	int getNumPreferredPenetrationDirections() {
		return 0;
	}

	@override
	void getPreferredPenetrationDirection(int index, Vector3 penetrationVector) {
		throw 'InternalError()';
	}
	
}
