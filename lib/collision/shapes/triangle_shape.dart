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
import "package:bullet_physics/collision/shapes/polyhedral_convex_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class TriangleShape extends PolyhedralConvexShape {
	
	final List<Vector3> vertices1/*[3]*/ = [ Vector3.zero(), Vector3.zero(), Vector3.zero() ];
	
	TriangleShape([Vector3? p0, Vector3? p1, Vector3? p2]) {
    if(p0 != null){
		  vertices1[0].setFrom(p0);
    }
    if(p1 != null){
		  vertices1[1].setFrom(p1);
    }
    if(p2 != null){
		  vertices1[2].setFrom(p2);
    }
	}
	
	// JAVA NOTE: added
	void init(Vector3 p0, Vector3 p1, Vector3 p2) {
		vertices1[0].setFrom(p0);
		vertices1[1].setFrom(p1);
		vertices1[2].setFrom(p2);
	}

	@override
	int getNumVertices() {
		return 3;
	}

	Vector3 getVertexPtr(int index) {
		return vertices1[index];
	}
	
	@override
	void getVertex(int index, Vector3 vert) {
		vert.setFrom(vertices1[index]);
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.triangleShapeProxyType;
	}

	@override
	int getNumEdges() {
		return 3;
	}

	@override
	void getEdge(int i, Vector3 pa, Vector3 pb) {
		getVertex(i, pa);
		getVertex((i + 1) % 3, pb);
	}

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
//		btAssert(0);
		getAabbSlow(t, aabbMin, aabbMax);
	}

	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 dir, Vector3 out) {
		Vector3 dots = Vector3.zero();
		dots.setValues(dir.dot(vertices1[0]), dir.dot(vertices1[1]), dir.dot(vertices1[2]));
		out.setFrom(vertices1[VectorUtil.maxAxis(dots)]);
		return out;
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		Vector3 dots = Vector3.zero();

		for (int i = 0; i < numVectors; i++) {
			Vector3 dir = vectors[i];
			dots.setValues(dir.dot(vertices1[0]), dir.dot(vertices1[1]), dir.dot(vertices1[2]));
			supportVerticesOut[i].setFrom(vertices1[VectorUtil.maxAxis(dots)]);
		}
	}

	@override
	void getPlane(Vector3 planeNormal, Vector3 planeSupport, int i) {
		getPlaneEquation(i,planeNormal,planeSupport);
	}

	@override
	int getNumPlanes() {
		return 1;
	}

	void calcNormal(Vector3 normal) {
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		tmp1.sub2(vertices1[1],vertices1[0]);
		tmp2.sub2(vertices1[2],vertices1[0]);

		normal.cross2(tmp1, tmp2);
		normal.normalize();
	}

	void getPlaneEquation(int i, Vector3 planeNormal, Vector3 planeSupport) {
		calcNormal(planeNormal);
		planeSupport.setFrom(vertices1[0]);
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		assert (false);
		inertia.setValues(0, 0, 0);
	}
	
	@override
	bool isInside(Vector3 pt, double tolerance) {
		Vector3 normal = Vector3.zero();
		calcNormal(normal);
		// distance to plane
		double dist = pt.dot(normal);
		double planeconst = vertices1[0].dot(normal);
		dist -= planeconst;
		if (dist >= -tolerance && dist <= tolerance) {
			// inside check on edge-planes
			int i;
			for (i = 0; i < 3; i++) {
				Vector3 pa = Vector3.zero(), pb = Vector3.zero();
				getEdge(i, pa, pb);
				Vector3 edge = Vector3.zero();
				edge.sub2(pb,pa);
				Vector3 edgeNormal = Vector3.zero();
				edgeNormal.cross2(edge, normal);
				edgeNormal.normalize();
				/*double*/ dist = pt.dot(edgeNormal);
				double edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance) {
					return false;
				}
			}

			return true;
		}

		return false;
	}

	@override
	String getName() {
		return "Triangle";
	}

	@override
	int getNumPreferredPenetrationDirections() {
		return 2;
	}

	@override
	void getPreferredPenetrationDirection(int index, Vector3 penetrationVector) {
		calcNormal(penetrationVector);
		if (index != 0) {
			penetrationVector.scale(-1);
		}
	}

}
