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

import "package:bullet_physics/extras/gimpact/bvh_tree_node_array.dart";
import "package:bullet_physics/extras/gimpact/bvh_data_array.dart";
import "package:bullet_physics/linearmath/aabb.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class BvhTree {

	int numNodes = 0;
	BvhTreeNodeArray nodeArray = BvhTreeNodeArray();
	
	int _calcSplittingAxis(BvhDataArray primitiveBoxes, int startIndex, int endIndex) {
		Vector3 means = Vector3.zero();
		means.setValues(0, 0, 0);
		Vector3 variance = Vector3.zero();
		variance.setValues(0, 0, 0);

		int numIndices = endIndex - startIndex;

		Vector3 center = Vector3.zero();
		Vector3 diff2 = Vector3.zero();

		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		for (int i=startIndex; i<endIndex; i++) {
			primitiveBoxes.getBoundMax(i, tmp1);
			primitiveBoxes.getBoundMin(i, tmp2);
			center.add2(tmp1, tmp2);
			center.scale(0.5);
			means.add(center);
		}
		means.scale(1 /numIndices);

		for (int i=startIndex; i<endIndex; i++) {
			primitiveBoxes.getBoundMax(i, tmp1);
			primitiveBoxes.getBoundMin(i, tmp2);
			center.add2(tmp1, tmp2);
			center.scale(0.5);
			diff2.sub2(center, means);
			VectorUtil.mul(diff2, diff2, diff2);
			variance.add(diff2);
		}
		variance.scale(1 / (numIndices - 1));

		return VectorUtil.maxAxis(variance);
	}

	int _sortAndCalcSplittingIndex(BvhDataArray primitiveBoxes, int startIndex, int endIndex, int splitAxis) {
		int splitIndex = startIndex;
		int numIndices = endIndex - startIndex;

		// average of centers
		double splitValue = 0.0;

		Vector3 means = Vector3.zero();
		means.setValues(0, 0, 0);

		Vector3 center = Vector3.zero();

		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();

		for (int i = startIndex; i < endIndex; i++) {
			primitiveBoxes.getBoundMax(i, tmp1);
			primitiveBoxes.getBoundMin(i, tmp2);
			center.add2(tmp1, tmp2);
			center.scale(0.5);
			means.add(center);
		}
		means.scale(1 / numIndices);

		splitValue = VectorUtil.getCoord(means, splitAxis);

		// sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
		for (int i = startIndex; i < endIndex; i++) {
			primitiveBoxes.getBoundMax(i, tmp1);
			primitiveBoxes.getBoundMin(i, tmp2);
			center.add2(tmp1, tmp2);
			center.scale(0.5);

			if (VectorUtil.getCoord(center, splitAxis) > splitValue) {
				// swap
				primitiveBoxes.swap(i, splitIndex);
				//swapLeafNodes(i,splitIndex);
				splitIndex++;
			}
		}

		// if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
		// otherwise the tree-building might fail due to stack-overflows in certain cases.
		// unbalanced1 is unsafe: it can cause stack overflows
		//bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

		// unbalanced2 should work too: always use center (perfect balanced trees)
		//bool unbalanced2 = true;

		// this should be safe too:
		int rangeBalancedIndices = numIndices ~/ 3;
		bool unbalanced = ((splitIndex <= (startIndex + rangeBalancedIndices)) || (splitIndex >= (endIndex - 1 - rangeBalancedIndices)));

		if (unbalanced) {
			splitIndex = startIndex + (numIndices >> 1);
		}

		bool unbal = (splitIndex == startIndex) || (splitIndex == (endIndex));
		assert (!unbal);

		return splitIndex;
	}

	void _buildSubTree(BvhDataArray primitiveBoxes, int startIndex, int endIndex) {
		int curIndex = numNodes;
		numNodes++;

		assert ((endIndex - startIndex) > 0);

		if ((endIndex - startIndex) == 1) {
			// We have a leaf node
			//setNodeBound(curIndex,primitiveBoxes[startIndex].m_bound);
			//m_node_array[curIndex].setDataIndex(primitiveBoxes[startIndex].m_data);
			nodeArray.setFromData(curIndex, primitiveBoxes, startIndex);

			return;
		}
		// calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

		// split axis
		int splitIndex = _calcSplittingAxis(primitiveBoxes, startIndex, endIndex);

		splitIndex = _sortAndCalcSplittingIndex(primitiveBoxes, startIndex, endIndex, splitIndex);

		//calc this node bounding box

		AABB nodeBound = AABB();
		AABB tmpAABB = AABB();

		nodeBound.invalidate();

		for (int i=startIndex; i<endIndex; i++) {
			primitiveBoxes.getBound(i, tmpAABB);
			nodeBound.merge(tmpAABB);
		}

		setNodeBound(curIndex, nodeBound);

		// build left branch
		_buildSubTree(primitiveBoxes, startIndex, splitIndex);

		// build right branch
		_buildSubTree(primitiveBoxes, splitIndex, endIndex);

		nodeArray.setEscapeIndex(curIndex, numNodes - curIndex);
	}
	
	void buildTree(BvhDataArray primitiveBoxes) {
		// initialize node count to 0
		numNodes = 0;
		// allocate nodes
		nodeArray.resize(primitiveBoxes.length*2);

		_buildSubTree(primitiveBoxes, 0, primitiveBoxes.length);
	}

	void clearNodes() {
		nodeArray.clear();
		numNodes = 0;
	}

	int getNodeCount() {
		return numNodes;
	}

	/**
	 * Tells if the node is a leaf.
	 */
	bool isLeafNode(int nodeindex) {
		return nodeArray.isLeafNode(nodeindex);
	}

	int getNodeData(int nodeindex) {
		return nodeArray.getDataIndex(nodeindex);
	}

	void getNodeBound(int nodeindex, AABB bound) {
		nodeArray.getBound(nodeindex, bound);
	}

	void setNodeBound(int nodeindex, AABB bound) {
		nodeArray.setBound(nodeindex, bound);
	}

	int getLeftNode(int nodeindex) {
		return nodeindex + 1;
	}

	int getRightNode(int nodeindex) {
		if (nodeArray.isLeafNode(nodeindex + 1)) {
			return nodeindex + 2;
		}
		return nodeindex + 1 + nodeArray.getEscapeIndex(nodeindex + 1);
	}

	int getEscapeNodeIndex(int nodeindex) {
		return nodeArray.getEscapeIndex(nodeindex);
	}

	BvhTreeNodeArray getNodePointer(){
		return nodeArray;
	}
	
}
