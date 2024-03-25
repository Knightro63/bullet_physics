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

import "package:bullet_physics/collision/shapes/striding_mesh_interface.dart";
import "package:bullet_physics/collision/shapes/vertex_data.dart";
import "package:bullet_physics/extras/gimpact/primitive_manager_base.dart";
import "package:bullet_physics/extras/gimpact/primitive_triangle.dart";
import "package:bullet_physics/extras/gimpact/triangle_shape_ex.dart";
import "package:bullet_physics/linearmath/aabb.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class TrimeshPrimitiveManager extends PrimitiveManagerBase {
	double margin = 0;
	StridingMeshInterface? meshInterface;
	final Vector3 scale = Vector3.zero();
	int part = 0;
	int lockCount = 0;

	final List<int> _tmpIndices = List.filled(3,0);

	VertexData? _vertexData;
	
	TrimeshPrimitiveManager() {
		meshInterface = null;
		part = 0;
		margin = 0.01;
		scale.setValues(1, 1, 1);
		lockCount = 0;
	}

	TrimeshPrimitiveManager.fromManager(TrimeshPrimitiveManager manager) {
		meshInterface = manager.meshInterface;
		part = manager.part;
		margin = manager.margin;
		scale.setFrom(manager.scale);
		lockCount = 0;
	}

	TrimeshPrimitiveManager.fromMeshInterface(this.meshInterface, this.part) {
		meshInterface?.getScaling(scale);
		margin = 0.1;
		lockCount = 0;
	}

	void lock() {
		if (lockCount > 0) {
			lockCount++;
			return;
		}
		_vertexData = meshInterface?.getLockedReadOnlyVertexIndexBase(part);

		lockCount = 1;
	}

	void unlock() {
		if (lockCount == 0) {
			return;
		}
		if (lockCount > 1) {
			--lockCount;
			return;
		}
		meshInterface?.unLockReadOnlyVertexBase(part);
		_vertexData = null;
		lockCount = 0;
	}
	
	@override
	bool isTrimesh() {
		return true;
	}

	@override
	int getPrimitiveCount() {
		return (_vertexData?.getIndexCount() ?? 0)~/3;
	}

	int getVertexCount() {
		return _vertexData?.getVertexCount() ?? 0;
	}

	void getIndices(int faceIndex, List<int> out) {
		out[0] = _vertexData?.getIndex(faceIndex*3+0) ?? 0;
		out[1] = _vertexData?.getIndex(faceIndex*3+1) ?? 0;
		out[2] = _vertexData?.getIndex(faceIndex*3+2) ?? 0;
	}

	void getVertex(int vertexIndex, Vector3 vertex) {
		_vertexData?.getVertex(vertexIndex, vertex);
		VectorUtil.mul(vertex, vertex, scale);
	}
	
	@override
	void getPrimitiveBox(int primIndex, AABB primBox) {
		PrimitiveTriangle triangle = PrimitiveTriangle();
		getPrimitiveTriangle(primIndex, triangle);
		primBox.calcFromTriangleMargin(
				triangle.vertices[0],
				triangle.vertices[1], triangle.vertices[2], triangle.margin);
	}

	@override
	void getPrimitiveTriangle(int primIndex, PrimitiveTriangle triangle) {
		getIndices(primIndex, _tmpIndices);
		getVertex(_tmpIndices[0], triangle.vertices[0]);
		getVertex(_tmpIndices[1], triangle.vertices[1]);
		getVertex(_tmpIndices[2], triangle.vertices[2]);
		triangle.margin = margin;
	}

	void getBulletTriangle(int primIndex, TriangleShapeEx? triangle) {
    if(triangle == null) return;
		getIndices(primIndex, _tmpIndices);
		getVertex(_tmpIndices[0], triangle.vertices1[0]);
		getVertex(_tmpIndices[1], triangle.vertices1[1]);
		getVertex(_tmpIndices[2], triangle.vertices1[2]);
		triangle.setMargin(margin);
	}
}
