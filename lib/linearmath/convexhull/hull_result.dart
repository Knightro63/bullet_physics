/*
 * Dart port of Bullet (c) 2024 @Knightro
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

import 'package:bullet_physics/utils/int_array_list.dart';
import 'package:bullet_physics/utils/object_array_list.dart';
import 'package:vector_math/vector_math.dart';

/**
 * Contains resulting polygonal representation.<p>
 * 
 * Depending on the {@link #polygons} flag, array of indices consists of:<br>
 * <b>for triangles:</b> indices are array indexes into the vertex list<br>
 * <b>for polygons:</b> indices are in the form (number of points in face) (p1, p2, p3, ...)
 * 
 * @author jezek2
 */
class HullResult {
	
	/** True if indices represents polygons, false indices are triangles. */
	bool polygons = true;
	
	/** Number of vertices in the output hull. */
	int numOutputVertices = 0;
	
	/** Array of vertices. */
	final ObjectArrayList<Vector3> outputVertices = ObjectArrayList();
	
	/** Number of faces produced. */
	int numFaces = 0;
	
	/** Total number of indices. */
	int numIndices = 0;
	
	/** Array of indices. */
	final IntArrayList indices = IntArrayList();
}
