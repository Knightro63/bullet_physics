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

import 'package:bullet_physics/collision/shapes/internal_triangle_index_callback.dart';
import 'package:bullet_physics/collision/shapes/vertex_data.dart';
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

abstract class StridingMeshInterface {

	final Vector3 scaling = Vector3(1, 1, 1);
	
	void internalProcessAllTriangles(InternalTriangleIndexCallback callback, Vector3 aabbMin, Vector3 aabbMax) {
		int graphicssubparts = getNumSubParts();
		List<Vector3> triangle/*[3]*/ = [ Vector3.zero(), Vector3.zero(), Vector3.zero() ];

		Vector3 meshScaling = getScaling(Vector3.zero());

		for (int part=0; part<graphicssubparts; part++) {
			VertexData data = getLockedReadOnlyVertexIndexBase(part);

			for (int i=0, cnt = data.getIndexCount()~/3; i<cnt; i++) {
				data.getTriangle(i*3, meshScaling, triangle);
				callback.internalProcessTriangleIndex(triangle, part, i);
			}

			unLockReadOnlyVertexBase(part);
		}
	}
	
	void calculateAabbBruteForce(Vector3 aabbMin, Vector3 aabbMax) {
		// first calculate the total aabb for all triangles
		_AabbCalculationCallback aabbCallback = _AabbCalculationCallback();
		aabbMin.setValues(-1e30, -1e30, -1e30);
		aabbMax.setValues(1e30, 1e30, 1e30);
		internalProcessAllTriangles(aabbCallback, aabbMin, aabbMax);

		aabbMin.setFrom(aabbCallback.aabbMin);
		aabbMax.setFrom(aabbCallback.aabbMax);
	}
	
	/**
	 * Get read and write access to a subpart of a triangle mesh.
	 * This subpart has a continuous array of vertices and indices.
	 * In this way the mesh can be handled as chunks of memory with striding
	 * very similar to OpenGL vertexarray support.
	 * Make a call to unLockVertexBase when the read and write access is finished.
	 */
	VertexData getLockedVertexIndexBase(int subpart/*=0*/);

	VertexData getLockedReadOnlyVertexIndexBase(int subpart/*=0*/);

	/**
	 * unLockVertexBase finishes the access to a subpart of the triangle mesh.
	 * Make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished.
	 */
	void unLockVertexBase(int subpart);

	void unLockReadOnlyVertexBase(int subpart);

	/**
	 * getNumSubParts returns the number of seperate subparts.
	 * Each subpart has a continuous array of vertices and indices.
	 */
	int getNumSubParts();

	void preallocateVertices(int numverts);
	void preallocateIndices(int numindices);

	Vector3 getScaling(Vector3 out) {
		out.setFrom(scaling);
		return out;
	}
	
	void setScaling(Vector3 scaling) {
		this.scaling.setFrom(scaling);
	}
	
}

class _AabbCalculationCallback extends InternalTriangleIndexCallback {
  final Vector3 aabbMin = Vector3(1e30, 1e30, 1e30);
  final Vector3 aabbMax = Vector3(-1e30, -1e30, -1e30);
  @override
  void internalProcessTriangleIndex(List<Vector3> triangle, int partId, int triangleIndex) {
    VectorUtil.setMin(aabbMin, triangle[0]);
    VectorUtil.setMax(aabbMax, triangle[0]);
    VectorUtil.setMin(aabbMin, triangle[1]);
    VectorUtil.setMax(aabbMax, triangle[1]);
    VectorUtil.setMin(aabbMin, triangle[2]);
    VectorUtil.setMax(aabbMax, triangle[2]);
  }
}