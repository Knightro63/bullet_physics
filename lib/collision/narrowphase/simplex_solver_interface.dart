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
import 'package:vector_math/vector_math.dart';

/**
 * SimplexSolverInterface can incrementally calculate distance between origin and
 * up to 4 vertices. Used by GJK or Linear Casting. Can be implemented by the
 * Johnson-algorithm or alternative approaches based on voronoi regions or barycentric
 * coordinates.
 * 
 * @author jezek2
 */
abstract class SimplexSolverInterface {
	void reset();
	void addVertex(Vector3 w, Vector3 p, Vector3 q);
	bool closest(Vector3 v);
	double maxVertex();
	bool fullSimplex();
	int getSimplex(List<Vector3> pBuf, List<Vector3> qBuf, List<Vector3> yBuf);
	bool inSimplex(Vector3 w);
	void backupClosest(Vector3 v);
	bool emptySimplex();
	void computePoints(Vector3 p1, Vector3 p2);
	int numVertices();
}
