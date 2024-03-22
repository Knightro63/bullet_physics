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

import 'dart:typed_data';

import 'package:bullet_physics/collision/shapes/byte_buffer_vertex_data.dart';
import 'package:bullet_physics/collision/shapes/indexed_mesh.dart';
import 'package:bullet_physics/collision/shapes/scalar_type.dart';
import 'package:bullet_physics/collision/shapes/striding_mesh_interface.dart';
import 'package:bullet_physics/collision/shapes/vertex_data.dart';
import 'package:bullet_physics/utils/object_array_list.dart';

class TriangleIndexVertexArray extends StridingMeshInterface {
	ObjectArrayList<IndexedMesh> indexedMeshes = ObjectArrayList();//List<IndexedMesh>();
	ByteBufferVertexData _data = ByteBufferVertexData();

	/**
	 * Just to be backwards compatible.
	 */
	TriangleIndexVertexArray([int numTriangles = 0, Uint8List? triangleIndexBase, int triangleIndexStride = 0, int numVertices = 0, Float64List? vertexBase, int vertexStride = 0]) {
		IndexedMesh mesh = IndexedMesh();

		mesh.numTriangles = numTriangles;
		mesh.triangleIndexBase = triangleIndexBase;
		mesh.triangleIndexStride = triangleIndexStride;
		mesh.numVertices = numVertices;
		mesh.vertexBase = vertexBase;
		mesh.vertexStride = vertexStride;

		addIndexedMesh(mesh);
	}

	void addIndexedMesh(IndexedMesh mesh, [ScalarType indexType = ScalarType.integer]) {
		indexedMeshes.add(mesh);
		indexedMeshes.getQuick(indexedMeshes.size - 1)?.indexType = indexType;
	}
	
	@override
	VertexData getLockedVertexIndexBase(int subpart) {
		assert (subpart < getNumSubParts());

		IndexedMesh? mesh = indexedMeshes.getQuick(subpart);

		_data.vertexCount = mesh?.numVertices ?? 0;
		_data.vertexData = mesh?.vertexBase;
		//#ifdef BT_USE_DOUBLE_PRECISION
		//type = PHY_DOUBLE;
		//#else
		_data.vertexType = ScalarType.double;
		//#endif
		_data.vertexStride = (mesh?.vertexStride ?? 0);

		_data.indexCount = (mesh?.numTriangles ?? 0)*3;

		_data.indexData = mesh?.triangleIndexBase;
		_data.indexStride = (mesh?.triangleIndexStride ?? 0)~/3;
		_data.indexType = mesh?.indexType;
		return _data;
	}

	@override
	VertexData getLockedReadOnlyVertexIndexBase(int subpart) {
		return getLockedVertexIndexBase(subpart);
	}

	/**
	 * unLockVertexBase finishes the access to a subpart of the triangle mesh.
	 * Make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished.
	 */
	@override
	void unLockVertexBase(int subpart) {
		_data.vertexData = null;
		_data.indexData = null;
	}

	@override
	void unLockReadOnlyVertexBase(int subpart) {
		unLockVertexBase(subpart);
	}

	/**
	 * getNumSubParts returns the number of seperate subparts.
	 * Each subpart has a continuous array of vertices and indices.
	 */
	@override
	int getNumSubParts() {
		return indexedMeshes.size;
	}

	ObjectArrayList<IndexedMesh> getIndexedMeshArray() {
		return indexedMeshes;
	}
	
	@override
	void preallocateVertices(int numverts) {
	}

	@override
	void preallocateIndices(int numindices) {
	}

}
