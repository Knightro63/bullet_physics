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

import "package:bullet_physics/collision/shapes/polyhedral_convex_shape.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';
import 'dart:math';

class ConvexHullShape extends PolyhedralConvexShape {
	final ObjectArrayList<Vector3> _points = ObjectArrayList();
	
	/**
	 * TODO: This constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive double (x,y,z), the striding defines the number of bytes between each point, in memory.
	 * It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
	 * ConvexHullShape make an internal copy of the points.
	 */
	// TODO: make better constuctors (ByteBuffer, etc.)
	ConvexHullShape(List<Vector3> points) {
		// JAVA NOTE: rewritten
		
		for (int i=0; i<points.length; i++) {
			_points.add(Vector3.copy(points.getQuick(i)));
		}
		
		recalcLocalAabb();
	}

	@override
	void setLocalScaling(Vector3 scaling) {
		localScaling.setFrom(scaling);
		recalcLocalAabb();
	}
	
	void addPoint(Vector3 point) {
		_points.add(Vector3.copy(point));
		recalcLocalAabb();
	}

	ObjectArrayList<Vector3> getPoints() {
		return _points;
	}

	int getNumPoints() {
		return _points.size;
	}

	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec0, Vector3 out) {
		Vector3 supVec = out;
		supVec.setValues(0, 0, 0);
		double newDot, maxDot = -1e30;

		Vector3 vec = Vector3.copy(vec0);
		double lenSqr = vec.length2;
		if (lenSqr < 0.0001) {
			vec.setValues(1, 0, 0);
		}
		else {
			double rlen = 1 / sqrt(lenSqr);
			vec.scale(rlen);
		}


		Vector3 vtx = Vector3.zero();
		for (int i = 0; i < _points.size; i++) {
			VectorUtil.mul(vtx, _points.getQuick(i) ?? Vector3.zero(), localScaling);

			newDot = vec.dot(vtx);
			if (newDot > maxDot) {
				maxDot = newDot;
				supVec.setFrom(vtx);
			}
		}
		return out;
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		double newDot;

		// JAVA NOTE: rewritten as code used W coord for temporary usage in Vector3
		// TODO: optimize it
		List<double> wcoords = List.filled(numVectors, 0);

		// use 'w' component of supportVerticesOut?
		{
			for (int i = 0; i < numVectors; i++) {
				//supportVerticesOut[i][3] = btScalar(-1e30);
				wcoords[i] = -1e30;
			}
		}
		Vector3 vtx = Vector3.zero();
		for (int i = 0; i < _points.size; i++) {
			VectorUtil.mul(vtx, _points.getQuick(i) ?? Vector3.zero(), localScaling);

			for (int j = 0; j < numVectors; j++) {
				Vector3 vec = vectors[j];

				newDot = vec.dot(vtx);
				//if (newDot > supportVerticesOut[j][3])
				if (newDot > wcoords[j]) {
					// WARNING: don't swap next lines, the w component would get overwritten!
					supportVerticesOut[j].setFrom(vtx);
					//supportVerticesOut[j][3] = newDot;
					wcoords[j] = newDot;
				}
			}
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

	/**
	 * Currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection.
	 * Please note that you can debug-draw ConvexHullShape with the Raytracer Demo.
	 */
	@override
	int getNumVertices() {
		return _points.size;
	}

	@override
	int getNumEdges() {
		return _points.size;
	}

	@override
	void getEdge(int i, Vector3 pa, Vector3 pb) {
		int index0 = i % _points.size;
		int index1 = (i + 1) % _points.size;
		VectorUtil.mul(pa, _points.getQuick(index0) ?? Vector3.zero(), localScaling);
		VectorUtil.mul(pb, _points.getQuick(index1) ?? Vector3.zero(), localScaling);
	}

	@override
	void getVertex(int i, Vector3 vtx) {
		VectorUtil.mul(vtx, _points.getQuick(i) ?? Vector3.zero(), localScaling);
	}

	@override
	int getNumPlanes() {
		return 0;
	}

	@override
	void getPlane(Vector3 planeNormal, Vector3 planeSupport, int i) {
		assert (false);
	}

	@override
	bool isInside(Vector3 pt, double tolerance) {
		assert( false);
		return false;
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.convexHullShapeProxytype;
	}

	@override
	String getName() {
		return "Convex";
	}
}
