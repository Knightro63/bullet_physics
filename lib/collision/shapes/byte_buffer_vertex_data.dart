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

import 'dart:typed_data';
import 'package:bullet_physics/collision/shapes/scalar_type.dart';
import 'package:bullet_physics/collision/shapes/vertex_data.dart';
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class ByteBufferVertexData extends VertexData {

	Float64List? vertexData;
	int vertexCount = 0;
	int vertexStride = 0;
	ScalarType? vertexType;

	Uint8List? indexData;
	int indexCount = 0;
	int indexStride = 0;
	ScalarType? indexType;

	@override
	int getVertexCount() {
		return vertexCount;
	}

	@override
	int getIndexCount() {
		return indexCount;
	}

	@override
	Vector3 getVertex(int idx, Vector3 out) {
		int off = idx*vertexStride;
		out.x = vertexData?[off+4*0] ?? 0;
		out.y = vertexData?[off+4*1] ?? 0;
		out.z = vertexData?[off+4*2] ?? 0;
		return out;
	}

	@override
	void setVertexValues(int idx, double x, double y, double z) {
		int off = idx*vertexStride;
		vertexData?[off+4*0] = x;
		vertexData?[off+4*1] = y;
		vertexData?[off+4*2] = z;
	}

	@override
	int getIndex(int idx) {
		if (indexType == ScalarType.int) {
			return (indexData?[idx*indexStride] ?? 0) & 0xFFFF;
		}
		else if (indexType == ScalarType.integer) {
			return indexData?[idx*indexStride] ?? 0;
		}
		else {
			throw("indicies type must be int or integer");
		}
	}
}
