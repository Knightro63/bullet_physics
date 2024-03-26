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
import "package:bullet_physics/collision/shapes/convex_internal_shape.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class MinkowskiSumShape extends ConvexInternalShape {
	final Transform _transA = Transform();
	final Transform _transB = Transform();
	late ConvexShape _shapeA;
	late ConvexShape _shapeB;

	MinkowskiSumShape(ConvexShape shapeA, ConvexShape shapeB) {
		_shapeA = shapeA;
		_shapeB = shapeB;
		_transA.setIdentity();
		_transB.setIdentity();
	}
	
	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		Vector3 tmp = Vector3.zero();
		Vector3 supVertexA = Vector3.zero();
		Vector3 supVertexB = Vector3.zero();

		// btVector3 supVertexA = m_transA(m_shapeA->localGetSupportingVertexWithoutMargin(-vec*m_transA.getBasis()));
		tmp.negateFrom(vec);
		MatrixUtil.transposeTransform(tmp, tmp, _transA.basis);
		_shapeA.localGetSupportingVertexWithoutMargin(tmp, supVertexA);
		_transA.transform(supVertexA);

		// btVector3 supVertexB = m_transB(m_shapeB->localGetSupportingVertexWithoutMargin(vec*m_transB.getBasis()));
		MatrixUtil.transposeTransform(tmp, vec, _transB.basis);
		_shapeB.localGetSupportingVertexWithoutMargin(tmp, supVertexB);
		_transB.transform(supVertexB);

		//return supVertexA - supVertexB;
		out.sub2(supVertexA, supVertexB);
		return out;
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		//todo: could make recursive use of batching. probably this shape is not used frequently.
		for (int i = 0; i < numVectors; i++) {
			localGetSupportingVertexWithoutMargin(vectors[i], supportVerticesOut[i]);
		}
	}

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		throw("Not supported yet.");
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.minkowskiSumShapeProxytype;
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		assert (false);
		inertia.setValues(0, 0, 0);
	}

	@override
	String getName() {
		return "MinkowskiSum";
	}
	
	@override
	double getMargin() {
		return _shapeA.getMargin() + _shapeB.getMargin();
	}

	void setTransformA(Transform transA) {
		_transA.copy(transA);
	}

	void setTransformB(Transform transB) {
		_transB.copy(transB);
	}

	void getTransformA(Transform dest) {
		dest.copy(_transA);
	}

	void getTransformB(Transform dest) {
		dest.copy(_transB);
	}
}
