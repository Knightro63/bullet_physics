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

import 'package:bullet_physics/collision/shapes/quantized_bvh_nodes.dart';

class BvhSubtreeInfo{ // implements Serializable 

	//static const int _serialVersionUID = 1;
	
	final /*unsigned*/ List<int> quantizedAabbMin = [0,0,0];//List.filled(3,0);
	final /*unsigned*/ List<int> quantizedAabbMax = [0,0,0];//List.filled(3,0);
	// points to the root of the subtree
	int rootNodeIndex = 0;
	int subtreeSize = 0;

	void setAabbFromQuantizeNode(QuantizedBvhNodes quantizedNodes, int nodeId) {
		quantizedAabbMin[0] = quantizedNodes.getQuantizedAabbMinWithIndex(nodeId, 0);
		quantizedAabbMin[1] = quantizedNodes.getQuantizedAabbMinWithIndex(nodeId, 1);
		quantizedAabbMin[2] = quantizedNodes.getQuantizedAabbMinWithIndex(nodeId, 2);
		quantizedAabbMax[0] = quantizedNodes.getQuantizedAabbMaxWithIndex(nodeId, 0);
		quantizedAabbMax[1] = quantizedNodes.getQuantizedAabbMaxWithIndex(nodeId, 1);
		quantizedAabbMax[2] = quantizedNodes.getQuantizedAabbMaxWithIndex(nodeId, 2);
	}

}
