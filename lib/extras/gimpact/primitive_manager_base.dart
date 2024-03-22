/*
 * Dart port of Bullet (c) 2024 @Knightro63
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



import "package:bullet_physics/extras/gimpact/primitive_triangle.dart";
import "package:bullet_physics/linearmath/aabb.dart";

/**
 * Prototype Base class for primitive classification.<p>
 * 
 * This class is a wrapper for primitive collections.<p>
 * 
 * This tells relevant info for the Bounding Box set classes, which take care of space classification.<p>
 * 
 * This class can manage Compound shapes and trimeshes, and if it is managing trimesh then the
 * Hierarchy Bounding Box classes will take advantage of primitive Vs Box overlapping tests for
 * getting optimal results and less Per Box compairisons.
 * 
 * @author jezek2
 */
abstract class PrimitiveManagerBase {
	/**
	 * Determines if this manager consist on only triangles, which special case will be optimized.
	 */
	bool isTrimesh();
	int getPrimitiveCount();
	void getPrimitiveBox(int primIndex, AABB primBox);
	
	/**
	 * Retrieves only the points of the triangle, and the collision margin.
	 */
	void getPrimitiveTriangle(int primIndex, PrimitiveTriangle triangle);
}
