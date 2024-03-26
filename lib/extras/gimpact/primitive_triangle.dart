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

import 'package:bullet_physics/extras/gimpact/clip_polygon.dart';
import 'package:bullet_physics/extras/gimpact/geometry_operations.dart';
import 'package:bullet_physics/extras/gimpact/triangle_contact.dart';
import "package:bullet_physics/linearmath/transform.dart";
import 'package:bullet_physics/linearmath/vector_util.dart';
import 'package:bullet_physics/utils/object_array_list.dart';
import 'package:vector_math/vector_math.dart';

class PrimitiveTriangle {
	final ObjectArrayList<Vector3> _tmpVecList1 = ObjectArrayList(TriangleContact.maxTriClipping);//List<Vector3>(TriangleContact.maxTriClipping);
	final ObjectArrayList<Vector3> _tmpVecList2 = ObjectArrayList(TriangleContact.maxTriClipping);//List<Vector3>(TriangleContact.maxTriClipping);
	final ObjectArrayList<Vector3> _tmpVecList3 = ObjectArrayList(TriangleContact.maxTriClipping);//List<Vector3>(TriangleContact.maxTriClipping);
	
	void _init(){
		for (int i=0; i<TriangleContact.maxTriClipping; i++) {
			_tmpVecList1.add(Vector3.zero());
			_tmpVecList2.add(Vector3.zero());
			_tmpVecList3.add(Vector3.zero());
		}
	}
	
	final List<Vector3> vertices = [Vector3.zero(),Vector3.zero(),Vector3.zero()];//Vector3[3];
	final Vector4 plane = Vector4.zero();
	double margin = 0.01;

	PrimitiveTriangle() {
    _init();
		for (int i=0; i<vertices.length; i++) {
			vertices[i] = Vector3.zero();
		}
	}
	
	void set(PrimitiveTriangle tri) {
		throw 'UnsupportedOperationException()';
	}
	
	void buildTriPlane() {
		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		Vector3 normal = Vector3.zero();
		tmp1.sub2(vertices[1], vertices[0]);
		tmp2.sub2(vertices[2], vertices[0]);
		normal.cross2(tmp1, tmp2);
		normal.normalize();

		plane.setValues(normal.x, normal.y, normal.z, vertices[0].dot(normal));
	}

	/**
	 * Test if triangles could collide.
	 */
	bool overlapTestConservative(PrimitiveTriangle other) {
		double totalMargin = margin + other.margin;
		// classify points on other triangle
		double dis0 = ClipPolygon.distancePointPlane(plane, other.vertices[0]) - totalMargin;
		double dis1 = ClipPolygon.distancePointPlane(plane, other.vertices[1]) - totalMargin;
		double dis2 = ClipPolygon.distancePointPlane(plane, other.vertices[2]) - totalMargin;

		if (dis0 > 0.0 && dis1 > 0.0 && dis2 > 0.0) {
			return false; // classify points on this triangle
		}
		
		dis0 = ClipPolygon.distancePointPlane(other.plane, vertices[0]) - totalMargin;

		dis1 = ClipPolygon.distancePointPlane(other.plane, vertices[1]) - totalMargin;

		dis2 = ClipPolygon.distancePointPlane(other.plane, vertices[2]) - totalMargin;

		if (dis0 > 0.0 && dis1 > 0.0 && dis2 > 0.0) {
			return false;
		}
		return true;
	}
	
	/**
	 * Calcs the plane which is paralele to the edge and perpendicular to the triangle plane.
	 * This triangle must have its plane calculated.
	 */
	void getEdgePlane(int edgeIndex, Vector4 plane) {
		Vector3 e0 = vertices[edgeIndex];
		Vector3 e1 = vertices[(edgeIndex + 1) % 3];

		Vector3 tmp = Vector3.zero();
		tmp.setValues(this.plane.x, this.plane.y, this.plane.z);

		GeometryOperations.edgePlane(e0, e1, tmp, plane);
	}

	void applyTransform(Transform? t) {
    if(t == null) return;
		t.transform(vertices[0]);
		t.transform(vertices[1]);
		t.transform(vertices[2]);
	}
	
	/**
	 * Clips the triangle against this.
	 * 
	 * @param clippedPoints must have MAX_TRI_CLIPPING size, and this triangle must have its plane calculated.
	 * @return the number of clipped points
	 */
	int clipTriangle(PrimitiveTriangle other, ObjectArrayList<Vector3> clippedPoints) {
		// edge 0
		ObjectArrayList<Vector3> tempPoints = _tmpVecList1;

		Vector4 edgeplane = Vector4.zero();

		getEdgePlane(0, edgeplane);

		int clippedCount = ClipPolygon.planeClipTriangle(edgeplane, other.vertices[0], other.vertices[1], other.vertices[2], tempPoints);

		if (clippedCount == 0) {
			return 0;
		}
		ObjectArrayList<Vector3> tempPoints1 = _tmpVecList2;

		// edge 1
		getEdgePlane(1, edgeplane);

		clippedCount = ClipPolygon.planeClipPolygon(edgeplane, tempPoints, clippedCount, tempPoints1);

		if (clippedCount == 0) {
			return 0; // edge 2
		}
		getEdgePlane(2, edgeplane);

		clippedCount = ClipPolygon.planeClipPolygon(edgeplane, tempPoints1, clippedCount, clippedPoints);

		return clippedCount;
	}
	
	/**
	 * Find collision using the clipping method.
	 * This triangle and other must have their triangles calculated.
	 */
	bool findTriangleCollisionClipMethod(PrimitiveTriangle other, TriangleContact contacts) {
		double margin = this.margin + other.margin;

		ObjectArrayList<Vector3> clippedPoints = _tmpVecList3;

		int clippedCount;
		//create planes
		// plane v vs U points

		TriangleContact contacts1 = TriangleContact();

		contacts1.separatingNormal.setFrom(plane);

		clippedCount = clipTriangle(other, clippedPoints);

		if (clippedCount == 0) {
			return false; // Reject
		}

		// find most deep interval face1
		contacts1.mergePoints(contacts1.separatingNormal, margin, clippedPoints, clippedCount);
		if (contacts1.pointCount == 0) {
			return false; // too far
		// Normal pointing to this triangle
		}
		contacts1.separatingNormal.x *= -1;
		contacts1.separatingNormal.y *= -1;
		contacts1.separatingNormal.z *= -1;

		// Clip tri1 by tri2 edges
		TriangleContact contacts2 = TriangleContact();
		contacts2.separatingNormal.setFrom(other.plane);

		clippedCount = other.clipTriangle(this, clippedPoints);

		if (clippedCount == 0) {
			return false; // Reject
		}

		// find most deep interval face1
		contacts2.mergePoints(contacts2.separatingNormal, margin, clippedPoints, clippedCount);
		if (contacts2.pointCount == 0) {
			return false; // too far

		// check most dir for contacts
		}
		if (contacts2.penetrationDepth < contacts1.penetrationDepth) {
			contacts.copyFrom(contacts2);
		}
		else {
			contacts.copyFrom(contacts1);
		}
		return true;
	}
	
}
