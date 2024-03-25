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

import 'dart:math';
import "package:bullet_physics/collision/shapes/convex_internal_shape.dart";
import "package:bullet_physics/linearmath/aabb_util2.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * PolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
 * 
 * @author jezek2
 */
abstract class PolyhedralConvexShape extends ConvexInternalShape {

	final List<Vector3> _directions = [
		Vector3( 1,  0,  0),
		Vector3( 0,  1,  0),
		Vector3( 0,  0,  1),
		Vector3(-1,  0,  0),
		Vector3( 0, -1,  0),
		Vector3( 0,  0, -1)
  ];

	final List<Vector3> _supporting = [
		Vector3(0, 0, 0),
		Vector3(0, 0, 0),
		Vector3(0, 0, 0),
		Vector3(0, 0, 0),
		Vector3(0, 0, 0),
		Vector3(0, 0, 0)
  ];
	
	final Vector3 localAabbMin = Vector3(1, 1, 1);
	final Vector3 localAabbMax = Vector3(-1, -1, -1);
	bool isLocalAabbValid = false;

//	/** optional Hull is for optional Separating Axis Test Hull collision detection, see Hull.cpp */
//	Hull optionalHull = null;
	
	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec0, Vector3 out) {
		int i;
		Vector3 supVec = out;
		supVec.setValues(0, 0, 0);

		double maxDot = -1e30;

		Vector3 vec = Vector3.copy(vec0);
		double lenSqr = vec.length2;
		if (lenSqr < 0.0001) {
			vec.setValues(1, 0, 0);
		}
		else {
			double rlen = 1 /sqrt(lenSqr);
			vec.scale(rlen);
		}

		Vector3 vtx = Vector3.zero();
		double newDot;

		for (i = 0; i < getNumVertices(); i++) {
			getVertex(i, vtx);
			newDot = vec.dot(vtx);
			if (newDot > maxDot) {
				maxDot = newDot;
				supVec = vtx;
			}
		}

		return out;
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		int i;

		Vector3 vtx = Vector3.zero();
		double newDot;

		// JAVA NOTE: rewritten as code used W coord for temporary usage in Vector3
		// TODO: optimize it
		List<double> wcoords = List.filled(numVectors, 0);

		for (i = 0; i < numVectors; i++) {
			// TODO: used w in vector3:
			//supportVerticesOut[i].w = -1e30f;
			wcoords[i] = -1e30;
		}

		for (int j = 0; j < numVectors; j++) {
			Vector3 vec = vectors[j];

			for (i = 0; i < getNumVertices(); i++) {
				getVertex(i, vtx);
				newDot = vec.dot(vtx);
				//if (newDot > supportVerticesOut[j].w)
				if (newDot > wcoords[j]) {
					//WARNING: don't swap next lines, the w component would get overwritten!
					supportVerticesOut[j].setFrom(vtx);
					//supportVerticesOut[j].w = newDot;
					wcoords[j] = newDot;
				}
			}
		}
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		// not yet, return box inertia

		double margin = getMargin();

		Transform ident = Transform();
		ident.setIdentity();
		Vector3 aabbMin = Vector3.zero(), aabbMax = Vector3.zero();
		getAabb(ident, aabbMin, aabbMax);

		Vector3 halfExtents = Vector3.zero();
		halfExtents.sub2(aabbMax,aabbMin);
		halfExtents.scale(0.5);

		double lx = 2 * (halfExtents.x + margin);
		double ly = 2 * (halfExtents.y + margin);
		double lz = 2 * (halfExtents.z + margin);
		double x2 = lx * lx;
		double y2 = ly * ly;
		double z2 = lz * lz;
		double scaledmass = mass * 0.08333333;

		inertia.setValues(y2 + z2, x2 + z2, x2 + y2);
		inertia.scale(scaledmass);
	}

	void _getNonvirtualAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax, double margin) {
		// lazy evaluation of local aabb
		assert (isLocalAabbValid);

		AabbUtil2.transformAabbLocal(localAabbMin, localAabbMax, margin, trans, aabbMin, aabbMax);
	}
	
	@override
	void getAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
		_getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
	}

	void polyhedralConvexShapeGetAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax) {
		_getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
	}

	void recalcLocalAabb() {
		isLocalAabbValid = true;

		//#if 1

		batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);

		for (int i=0; i<3; i++) {
			VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(_supporting[i], i) + collisionMargin);
			VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(_supporting[i + 3], i) - collisionMargin);
		}
		
		//#else
		//for (int i=0; i<3; i++) {
		//	Vector3 vec = Vector3.zero();
		//	vec.set(0, 0, 0);
		//	VectorUtil.setCoord(vec, i, 1);
		//	Vector3 tmp = localGetSupportingVertex(vec, Vector3.zero());
		//	VectorUtil.setCoord(localAabbMax, i, VectorUtil.getCoord(tmp, i) + collisionMargin);
		//	VectorUtil.setCoord(vec, i, -1);
		//	localGetSupportingVertex(vec, tmp);
		//	VectorUtil.setCoord(localAabbMin, i, VectorUtil.getCoord(tmp, i) - collisionMargin);
		//}
		//#endif
	}

	@override
	void setLocalScaling(Vector3 scaling) {
		super.setLocalScaling(scaling);
		recalcLocalAabb();
	}

	int getNumVertices();

	int getNumEdges();

	void getEdge(int i, Vector3 pa, Vector3 pb);

	void getVertex(int i, Vector3 vtx);

	int getNumPlanes();

	void getPlane(Vector3 planeNormal, Vector3 planeSupport, int i);
	
  //	int getIndex(int i) const = 0 ; 
	
	bool isInside(Vector3 pt, double tolerance);
}
