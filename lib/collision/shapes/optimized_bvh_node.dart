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

import 'package:vector_math/vector_math.dart';

class OptimizedBvhNode{ // implements Serializable 
	//static const int _serialVersionUID = 1;
	
	final Vector3 aabbMinOrg = Vector3.zero();
	final Vector3 aabbMaxOrg = Vector3.zero();

	int escapeIndex = 0;

	// for child nodes
	int subPart = 0;
	int triangleIndex = 0;
	
	void set(OptimizedBvhNode n) {
		aabbMinOrg.setFrom(n.aabbMinOrg);
		aabbMaxOrg.setFrom(n.aabbMaxOrg);
		escapeIndex = n.escapeIndex;
		subPart = n.subPart;
		triangleIndex = n.triangleIndex;
	}

}
