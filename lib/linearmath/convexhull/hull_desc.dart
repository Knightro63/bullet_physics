/*
 * Dart port of Bullet (c) 2024 @Knightro63
 *
 * Stan Melax Convex Hull Computation
 * Copyright (c) 2024 Stan Melax http://www.melax.com/
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
import 'hull_flags.dart';
import 'package:vector_math/vector_math.dart';
import '../../utils/object_array_list.dart';

/**
 * Describes point cloud data and other input for conversion to polygonal representation.
 * 
 * @author jezek2
 */
class HullDesc {
	
	HullDesc([int? flag, int? vcount, this.vertices, int? stride]) {
		flags = flag ?? HullFlags.defaultOrder;
		this.vcount = vcount ?? 0;
	}
	/** Flags to use when generating the convex hull, see {@link HullFlags}. */
	late int flags;
	
	/** Number of vertices in the input point cloud. */
	late int vcount;
	
	/** Array of vertices. */
	ObjectArrayList<Vector3>? vertices;
	
	/** Stride of each vertex, in bytes. */
	int vertexStride = 3*4;       
	
	/** Epsilon value for removing duplicates. This is a normalized value, if normalized bit is on. */
	double normalEpsilon = 0.001;
	
	/** Maximum number of vertices to be considered for the hull. */
	int maxVertices = 4096;

	/** Maximum number of faces to be considered for the hull. */
	int maxFaces = 4096;

	bool hasHullFlag(int flag) {
		if ((flags & flag) != 0) {
			return true;
		}
		return false;
	}
	void setHullFlag(int flag) {
		flags |= flag;
	}
	void clearHullFlag(int flag) {
		flags &= ~flag;
	}
}
