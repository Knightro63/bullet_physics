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

import 'package:bullet_physics/collision/broadphase/broadphase_native_type.dart';
import 'package:bullet_physics/collision/shapes/polyhedral_convex_shape.dart';
import 'package:vector_math/vector_math.dart';

class BUSimplex1to4 extends PolyhedralConvexShape {

	int numVertices = 0;
	List<Vector3> vertices = [Vector3.zero(),Vector3.zero(),Vector3.zero(),Vector3.zero()];

	BUSimplex1to4([Vector3? pt0, Vector3? pt1, Vector3? pt2, Vector3? pt3]) {
		addVertex(pt0);
		addVertex(pt1);
		addVertex(pt2);
		addVertex(pt3);
	}
	
	void reset() {
		numVertices = 0;
	}
	
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.tetrahedralShapeProxyType;
	}
	
	void addVertex(Vector3? pt) {
    if(pt == null) return;
		vertices[numVertices++] = pt;
		recalcLocalAabb();
	}

	
	@override
	int getNumVertices() {
		return numVertices;
	}

	@override
	int getNumEdges() {
		// euler formula, F-E+V = 2, so E = F+V-2

		switch (numVertices) {
			case 0: return 0;
			case 1: return 0;
			case 2: return 1;
			case 3: return 3;
			case 4: return 6;
		}

		return 0;
	}

	@override
	void getEdge(int i, Vector3 pa, Vector3 pb) {
		switch (numVertices) {
			case 2:
				pa.setFrom(vertices[0]);
				pb.setFrom(vertices[1]);
				break;
			case 3:
				switch (i) {
					case 0:
						pa.setFrom(vertices[0]);
						pb.setFrom(vertices[1]);
						break;
					case 1:
						pa.setFrom(vertices[1]);
						pb.setFrom(vertices[2]);
						break;
					case 2:
						pa.setFrom(vertices[2]);
						pb.setFrom(vertices[0]);
						break;
				}
				break;
			case 4:
				switch (i) {
					case 0:
						pa.setFrom(vertices[0]);
						pb.setFrom(vertices[1]);
						break;
					case 1:
						pa.setFrom(vertices[1]);
						pb.setFrom(vertices[2]);
						break;
					case 2:
						pa.setFrom(vertices[2]);
						pb.setFrom(vertices[0]);
						break;
					case 3:
						pa.setFrom(vertices[0]);
						pb.setFrom(vertices[3]);
						break;
					case 4:
						pa.setFrom(vertices[1]);
						pb.setFrom(vertices[3]);
						break;
					case 5:
						pa.setFrom(vertices[2]);
						pb.setFrom(vertices[3]);
						break;
				}
		}
	}

	@override
	void getVertex(int i, Vector3 vtx) {
		vtx.setFrom(vertices[i]);
	}

	@override
	int getNumPlanes() {
		switch (numVertices) {
			case 0: return 0;
			case 1: return 0;
			case 2: return 0;
			case 3: return 2;
			case 4: return 4;
		}
		return 0;
	}

	@override
	void getPlane(Vector3 planeNormal, Vector3 planeSupport, int i) {
	}
	
	int getIndex(int i) {
		return 0;
	}

	@override
	bool isInside(Vector3 pt, double tolerance) {
		return false;
	}

	String getName() {
		return "BU_Simplex1to4";
	}

}
