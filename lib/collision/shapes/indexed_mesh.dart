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

/**
 * IndexedMesh indexes into existing vertex and index arrays, in a similar way to
 * OpenGL's glDrawElements. Instead of the number of indices, we pass the number
 * of triangles.
 * 
 * @author jezek2
 */
class IndexedMesh {
	int numTriangles = 0;
	Uint8List? triangleIndexBase;
	int triangleIndexStride = 0;
	int numVertices = 0;
	Float64List? vertexBase;
	int vertexStride = 0;
	// The index type is set when adding an indexed mesh to the
	// TriangleIndexVertexArray, do not set it manually
	ScalarType? indexType;
}
