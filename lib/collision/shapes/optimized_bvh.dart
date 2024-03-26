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

import "package:bullet_physics/collision/shapes/bvh_subtree_info.dart";
import "package:bullet_physics/collision/shapes/internal_triangle_index_callback.dart";
import "package:bullet_physics/collision/shapes/node_overlap_callback.dart";
import "package:bullet_physics/collision/shapes/optimized_bvh_node.dart";
import "package:bullet_physics/collision/shapes/quantized_bvh_nodes.dart";
import "package:bullet_physics/collision/shapes/striding_mesh_interface.dart";
import "package:bullet_physics/collision/shapes/vertex_data.dart";
import "package:bullet_physics/linearmath/aabb_util2.dart";
import "package:bullet_physics/linearmath/misc_util.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';
import './traversal_mode.dart';

class OptimizedBvh{// implements Serializable 

	//static const int _serialVersionUID = 1;

	//final BulletStack stack = BulletStack.get();
	
	static const bool _debugTreeBuliding = false;
	static int _gStackDepth = 0;
	static int _gMaxStackDepth = 0;
	
	static int _maxIterations = 0;
	
	// Note: currently we have 16 bytes per quantized node
	static const int maxSubtreeSizeInBytes = 2048;

	// 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
	// actually) triangles each (since the sign bit is reserved
	static const int maxNumPartsInBits = 10;

	////////////////////////////////////////////////////////////////////////////

	final ObjectArrayList<OptimizedBvhNode> _leafNodes = ObjectArrayList();
	final ObjectArrayList<OptimizedBvhNode> _contiguousNodes = ObjectArrayList();

	QuantizedBvhNodes _quantizedLeafNodes = QuantizedBvhNodes();
	QuantizedBvhNodes _quantizedContiguousNodes = QuantizedBvhNodes();
	
	int _curNodeIndex = 0;

	// quantization data
	bool _useQuantization = false;
	final Vector3 _bvhAabbMin = Vector3.zero();
	final Vector3 _bvhAabbMax = Vector3.zero();
	final Vector3 _bvhQuantization = Vector3.zero();
	
	TraversalMode traversalMode = TraversalMode.stackless;
	final ObjectArrayList<BvhSubtreeInfo> SubtreeHeaders = ObjectArrayList();
	// This is only used for serialization so we don't have to add serialization directly to btAlignedObjectArray
	int subtreeHeaderCount = 0;

	// two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
	// this might be refactored into a virtual, it is usually not calculated at run-time
	void setInternalNodeAabbMin(int nodeIndex, Vector3 aabbMin) {
		if (_useQuantization) {
			_quantizedContiguousNodes.setQuantizedAabbMin(nodeIndex, quantizeWithClamp(aabbMin));
		}
		else {
			_contiguousNodes.getQuick(nodeIndex)?.aabbMinOrg.setFrom(aabbMin);
		}
	}

	void setInternalNodeAabbMax(int nodeIndex, Vector3 aabbMax) {
		if (_useQuantization) {
			_quantizedContiguousNodes.setQuantizedAabbMax(nodeIndex, quantizeWithClamp(aabbMax));
		}
		else {
			_contiguousNodes.getQuick(nodeIndex)?.aabbMaxOrg.setFrom(aabbMax);
		}
	}
	
	Vector3 getAabbMin(int nodeIndex) {
		if (_useQuantization) {
			Vector3 tmp = Vector3.zero();
			unQuantize(tmp, _quantizedLeafNodes.getQuantizedAabbMin(nodeIndex));
			return tmp;
		}

		// non-quantized
		return _leafNodes.getQuick(nodeIndex)?.aabbMinOrg ?? Vector3.zero();
	}

	Vector3 getAabbMax(int nodeIndex) {
		if (_useQuantization) {
			Vector3 tmp = Vector3.zero();
			unQuantize(tmp, _quantizedLeafNodes.getQuantizedAabbMax(nodeIndex));
			return tmp;
		}
		
		// non-quantized
		return _leafNodes.getQuick(nodeIndex)?.aabbMaxOrg ?? Vector3.zero();
	}
	
	void setQuantizationValues(Vector3? aabbMin, Vector3? aabbMax, [double quantizationMargin = 1]) {
		// enlarge the AABB to avoid division by zero when initializing the quantization values
		Vector3 clampValue = Vector3.zero();
    aabbMin ??= Vector3.zero();
    aabbMax ??= Vector3.zero();
		clampValue.setValues(quantizationMargin,quantizationMargin,quantizationMargin);
		_bvhAabbMin.sub2(aabbMin, clampValue);
		_bvhAabbMax.add2(aabbMax, clampValue);
		Vector3 aabbSize = Vector3.zero();
		aabbSize.sub2(_bvhAabbMax, _bvhAabbMin);
		_bvhQuantization.setValues(65535, 65535, 65535);
		VectorUtil.div(_bvhQuantization, _bvhQuantization, aabbSize);
	}
	
	void setInternalNodeEscapeIndex(int nodeIndex, int escapeIndex) {
		if (_useQuantization) {
			_quantizedContiguousNodes.setEscapeIndexOrTriangleIndex(nodeIndex, -escapeIndex);
		}
		else {
			_contiguousNodes.getQuick(nodeIndex)?.escapeIndex = escapeIndex;
		}
	}

	void mergeInternalNodeAabb(int nodeIndex, Vector3 newAabbMin, Vector3 newAabbMax) {
		if (_useQuantization) {
			int quantizedAabbMin;
			int quantizedAabbMax;

			quantizedAabbMin = quantizeWithClamp(newAabbMin);
			quantizedAabbMax = quantizeWithClamp(newAabbMax);
			for (int i = 0; i < 3; i++) {
				if (_quantizedContiguousNodes.getQuantizedAabbMinWithIndex(nodeIndex, i) > QuantizedBvhNodes.getCoord(quantizedAabbMin, i)) {
					_quantizedContiguousNodes.setQuantizedAabbMinWithIndex(nodeIndex, i, QuantizedBvhNodes.getCoord(quantizedAabbMin, i));
				}

				if (_quantizedContiguousNodes.getQuantizedAabbMaxWithIndex(nodeIndex, i) < QuantizedBvhNodes.getCoord(quantizedAabbMax, i)) {
					_quantizedContiguousNodes.setQuantizedAabbMaxWithIndex(nodeIndex, i, QuantizedBvhNodes.getCoord(quantizedAabbMax, i));
				}
			}
		}
		else {
			// non-quantized
			VectorUtil.setMin(_contiguousNodes.getQuick(nodeIndex)?.aabbMinOrg ?? Vector3.zero(), newAabbMin);
			VectorUtil.setMax(_contiguousNodes.getQuick(nodeIndex)?.aabbMaxOrg ?? Vector3.zero(), newAabbMax);
		}
	}
	
	void swapLeafNodes(int i, int splitIndex) {
		if (_useQuantization) {
			_quantizedLeafNodes.swap(i, splitIndex);
		}
		else {
			// JAVA NOTE: changing reference instead of copy
			OptimizedBvhNode? tmp = _leafNodes.getQuick(i);
			_leafNodes.setQuick(i, _leafNodes.getQuick(splitIndex));
			_leafNodes.setQuick(splitIndex, tmp);
		}
	}

	void assignInternalNodeFromLeafNode(int internalNode, int leafNodeIndex) {
		if (_useQuantization) {
			_quantizedContiguousNodes.set(internalNode, _quantizedLeafNodes, leafNodeIndex);
		}
		else {
			_contiguousNodes.getQuick(internalNode)?.set(_leafNodes.getQuick(leafNodeIndex)!);
		}
	}

	void build(StridingMeshInterface? triangles, bool useQuantizedAabbCompression, Vector3? aabbMin, Vector3? aabbMax) {
		_useQuantization = useQuantizedAabbCompression;

		// NodeArray	triangleNodes;

		int numLeafNodes = 0;

		if (_useQuantization) {
			// initialize quantization values
			setQuantizationValues(aabbMin, aabbMax);

			_QuantizedNodeTriangleCallback callback = _QuantizedNodeTriangleCallback(_quantizedLeafNodes, this);

			triangles?.internalProcessAllTriangles(callback, _bvhAabbMin,_bvhAabbMax);

			// now we have an array of leafnodes in m_leafNodes
			numLeafNodes = _quantizedLeafNodes.length;

			_quantizedContiguousNodes.resize(2 * numLeafNodes);
		}
		else {
			_NodeTriangleCallback callback = _NodeTriangleCallback(_leafNodes);

			Vector3 aabbMin = Vector3.zero();
			aabbMin.setValues(-1e30, -1e30, -1e30);
			Vector3 aabbMax = Vector3.zero();
			aabbMax.setValues(1e30, 1e30, 1e30);

			triangles?.internalProcessAllTriangles(callback, aabbMin, aabbMax);

			// now we have an array of leafnodes in m_leafNodes
			numLeafNodes = _leafNodes.size;

			// TODO: check
			//contiguousNodes.resize(2*numLeafNodes);
			MiscUtil.resizeObjectArray(_contiguousNodes, 2 * numLeafNodes, OptimizedBvhNode);
		}

		_curNodeIndex = 0;

		buildTree(0, numLeafNodes);

		//  if the entire tree is small then subtree size, we need to create a header info for the tree
		if (_useQuantization && SubtreeHeaders.isEmpty) {
			BvhSubtreeInfo subtree = BvhSubtreeInfo();
			SubtreeHeaders.add(subtree);

			subtree.setAabbFromQuantizeNode(_quantizedContiguousNodes, 0);
			subtree.rootNodeIndex = 0;
			subtree.subtreeSize = _quantizedContiguousNodes.isLeafNode(0) ? 1 : _quantizedContiguousNodes.getEscapeIndex(0);
		}

		// PCK: update the copy of the size
		subtreeHeaderCount = SubtreeHeaders.size;

		// PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
		_quantizedLeafNodes.clear();
		_leafNodes.clear();
	}
	
	void refit(StridingMeshInterface? meshInterface) {
		if (_useQuantization) {
			// calculate aabb
			Vector3 aabbMin = Vector3.zero(), aabbMax = Vector3.zero();
			meshInterface?.calculateAabbBruteForce(aabbMin, aabbMax);

			setQuantizationValues(aabbMin, aabbMax);

			updateBvhNodes(meshInterface, 0, _curNodeIndex, 0);

			// now update all subtree headers

			int i;
			for (i = 0; i < SubtreeHeaders.size; i++) {
				BvhSubtreeInfo? subtree = SubtreeHeaders.getQuick(i);
				subtree?.setAabbFromQuantizeNode(_quantizedContiguousNodes, subtree.rootNodeIndex);
			}

		}
		else {
			// JAVA NOTE: added for testing, it's too slow for practical use
			build(meshInterface, false, null, null);
		}
	}
	
	void refitPartial(StridingMeshInterface? meshInterface, Vector3 aabbMin, Vector3 aabbMax) {
		throw'UnsupportedOperationException()';
//		// incrementally initialize quantization values
//		assert (useQuantization);
//
//		btAssert(aabbMin.getX() > m_bvhAabbMin.getX());
//		btAssert(aabbMin.getY() > m_bvhAabbMin.getY());
//		btAssert(aabbMin.getZ() > m_bvhAabbMin.getZ());
//
//		btAssert(aabbMax.getX() < m_bvhAabbMax.getX());
//		btAssert(aabbMax.getY() < m_bvhAabbMax.getY());
//		btAssert(aabbMax.getZ() < m_bvhAabbMax.getZ());
//
//		///we should update all quantization values, using updateBvhNodes(meshInterface);
//		///but we only update chunks that overlap the given aabb
//
//		unsigned int	quantizedQueryAabbMin[3];
//		unsigned int	quantizedQueryAabbMax[3];
//
//		quantizeWithClamp(&quantizedQueryAabbMin[0],aabbMin);
//		quantizeWithClamp(&quantizedQueryAabbMax[0],aabbMax);
//
//		int i;
//		for (i=0;i<this->m_SubtreeHeaders.length;i++)
//		{
//			btBvhSubtreeInfo& subtree = m_SubtreeHeaders[i];
//
//			//PCK: unsigned instead of bool
//			unsigned overlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
//			if (overlap != 0)
//			{
//				updateBvhNodes(meshInterface,subtree.m_rootNodeIndex,subtree.m_rootNodeIndex+subtree.m_subtreeSize,i);
//
//				subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes[subtree.m_rootNodeIndex]);
//			}
//		}
	}
	
	void updateBvhNodes(StridingMeshInterface? meshInterface, int firstNode, int endNode, int index) {
		assert (_useQuantization);

		int curNodeSubPart = -1;

		List<Vector3> triangleVerts = [Vector3.zero(), Vector3.zero(), Vector3.zero()];
		Vector3 aabbMin = Vector3.zero(), aabbMax = Vector3.zero();
		Vector3 meshScaling = meshInterface?.getScaling(Vector3.zero()) ?? Vector3.zero();

		VertexData? data;

		for (int i = endNode - 1; i >= firstNode; i--) {
			QuantizedBvhNodes curNodes = _quantizedContiguousNodes;
			int curNodeId = i;

			if (curNodes.isLeafNode(curNodeId)) {
				// recalc aabb from triangle data
				int nodeSubPart = curNodes.getPartId(curNodeId);
				int nodeTriangleIndex = curNodes.getTriangleIndex(curNodeId);
				if (nodeSubPart != curNodeSubPart) {
					if (curNodeSubPart >= 0) {
						meshInterface?.unLockReadOnlyVertexBase(curNodeSubPart);
					}
					data = meshInterface?.getLockedReadOnlyVertexIndexBase(nodeSubPart);
				}
				//triangles->getLockedReadOnlyVertexIndexBase(vertexBase,numVerts,

				data?.getTriangle(nodeTriangleIndex*3, meshScaling, triangleVerts);

				aabbMin.setValues(1e30, 1e30, 1e30);
				aabbMax.setValues(-1e30, -1e30, -1e30);
				VectorUtil.setMin(aabbMin, triangleVerts[0]);
				VectorUtil.setMax(aabbMax, triangleVerts[0]);
				VectorUtil.setMin(aabbMin, triangleVerts[1]);
				VectorUtil.setMax(aabbMax, triangleVerts[1]);
				VectorUtil.setMin(aabbMin, triangleVerts[2]);
				VectorUtil.setMax(aabbMax, triangleVerts[2]);

				curNodes.setQuantizedAabbMin(curNodeId, quantizeWithClamp(aabbMin));
				curNodes.setQuantizedAabbMax(curNodeId, quantizeWithClamp(aabbMax));
			}
			else {
				// combine aabb from both children

				//quantizedContiguousNodes
				int leftChildNodeId = i + 1;

				int rightChildNodeId = _quantizedContiguousNodes.isLeafNode(leftChildNodeId) ? i + 2 : i + 1 + _quantizedContiguousNodes.getEscapeIndex(leftChildNodeId);

				for (int i2 = 0; i2 < 3; i2++) {
					curNodes.setQuantizedAabbMinWithIndex(curNodeId, i2, _quantizedContiguousNodes.getQuantizedAabbMinWithIndex(leftChildNodeId, i2));
					if (curNodes.getQuantizedAabbMinWithIndex(curNodeId, i2) > _quantizedContiguousNodes.getQuantizedAabbMinWithIndex(rightChildNodeId, i2)) {
						curNodes.setQuantizedAabbMinWithIndex(curNodeId, i2, _quantizedContiguousNodes.getQuantizedAabbMinWithIndex(rightChildNodeId, i2));
					}

					curNodes.setQuantizedAabbMaxWithIndex(curNodeId, i2, _quantizedContiguousNodes.getQuantizedAabbMaxWithIndex(leftChildNodeId, i2));
					if (curNodes.getQuantizedAabbMaxWithIndex(curNodeId, i2) < _quantizedContiguousNodes.getQuantizedAabbMaxWithIndex(rightChildNodeId, i2)) {
						curNodes.setQuantizedAabbMaxWithIndex(curNodeId, i2, _quantizedContiguousNodes.getQuantizedAabbMaxWithIndex(rightChildNodeId, i2));
					}
				}
			}
		}

		if (curNodeSubPart >= 0) {
			meshInterface?.unLockReadOnlyVertexBase(curNodeSubPart);
		}
	}
	
	void buildTree(int startIndex, int endIndex) {
		//#ifdef DEBUG_TREE_BUILDING
		if (_debugTreeBuliding) {
			_gStackDepth++;
			if (_gStackDepth > _gMaxStackDepth) {
				_gMaxStackDepth = _gStackDepth;
			}
		}
		//#endif //DEBUG_TREE_BUILDING

		int splitAxis, splitIndex, i;
		int numIndices = endIndex - startIndex;
		int curIndex = _curNodeIndex;

		assert (numIndices > 0);

		if (numIndices == 1) {
			//#ifdef DEBUG_TREE_BUILDING
			if (_debugTreeBuliding) {
				_gStackDepth--;
			}
			//#endif //DEBUG_TREE_BUILDING

			assignInternalNodeFromLeafNode(_curNodeIndex, startIndex);

			_curNodeIndex++;
			return;
		}
		// calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

		splitAxis = calcSplittingAxis(startIndex, endIndex);

		splitIndex = sortAndCalcSplittingIndex(startIndex, endIndex, splitAxis);

		int internalNodeIndex = _curNodeIndex;

		Vector3 tmp1 = Vector3.zero();
		tmp1.setValues(-1e30, -1e30, -1e30);
		setInternalNodeAabbMax(_curNodeIndex, tmp1);
		Vector3 tmp2 = Vector3.zero();
		tmp2.setValues(1e30, 1e30, 1e30);
		setInternalNodeAabbMin(_curNodeIndex, tmp2);

		for (i = startIndex; i < endIndex; i++) {
			mergeInternalNodeAabb(_curNodeIndex, getAabbMin(i), getAabbMax(i));
		}

		_curNodeIndex++;

		//internalNode->m_escapeIndex;

		int leftChildNodexIndex = _curNodeIndex;

		//build left child tree
		buildTree(startIndex, splitIndex);

		int rightChildNodexIndex = _curNodeIndex;
		// build right child tree
		buildTree(splitIndex, endIndex);

		//#ifdef DEBUG_TREE_BUILDING
		if (_debugTreeBuliding) {
			_gStackDepth--;
		}
		//#endif //DEBUG_TREE_BUILDING

		int escapeIndex = _curNodeIndex - curIndex;

		if (_useQuantization) {
			// escapeIndex is the number of nodes of this subtree
			int sizeQuantizedNode = QuantizedBvhNodes.getNodeSize();
			int treeSizeInBytes = escapeIndex * sizeQuantizedNode;
			if (treeSizeInBytes > maxSubtreeSizeInBytes) {
				updateSubtreeHeaders(leftChildNodexIndex, rightChildNodexIndex);
			}
		}

		setInternalNodeEscapeIndex(internalNodeIndex, escapeIndex);
	}

	bool testQuantizedAabbAgainstQuantizedAabb(int aabbMin1, int aabbMax1, int aabbMin2, int aabbMax2) {
		int aabbMin1_0 = QuantizedBvhNodes.getCoord(aabbMin1, 0);
		int aabbMin1_1 = QuantizedBvhNodes.getCoord(aabbMin1, 1);
		int aabbMin1_2 = QuantizedBvhNodes.getCoord(aabbMin1, 2);

		int aabbMax1_0 = QuantizedBvhNodes.getCoord(aabbMax1, 0);
		int aabbMax1_1 = QuantizedBvhNodes.getCoord(aabbMax1, 1);
		int aabbMax1_2 = QuantizedBvhNodes.getCoord(aabbMax1, 2);

		int aabbMin2_0 = QuantizedBvhNodes.getCoord(aabbMin2, 0);
		int aabbMin2_1 = QuantizedBvhNodes.getCoord(aabbMin2, 1);
		int aabbMin2_2 = QuantizedBvhNodes.getCoord(aabbMin2, 2);

		int aabbMax2_0 = QuantizedBvhNodes.getCoord(aabbMax2, 0);
		int aabbMax2_1 = QuantizedBvhNodes.getCoord(aabbMax2, 1);
		int aabbMax2_2 = QuantizedBvhNodes.getCoord(aabbMax2, 2);

		bool overlap = true;
		overlap = (aabbMin1_0 > aabbMax2_0 || aabbMax1_0 < aabbMin2_0) ? false : overlap;
		overlap = (aabbMin1_2 > aabbMax2_2 || aabbMax1_2 < aabbMin2_2) ? false : overlap;
		overlap = (aabbMin1_1 > aabbMax2_1 || aabbMax1_1 < aabbMin2_1) ? false : overlap;
		return overlap;
	}

	void updateSubtreeHeaders(int leftChildNodexIndex, int rightChildNodexIndex) {
		assert (_useQuantization);

		//btQuantizedBvhNode& leftChildNode = m_quantizedContiguousNodes[leftChildNodexIndex];
		int leftSubTreeSize = _quantizedContiguousNodes.isLeafNode(leftChildNodexIndex) ? 1 : _quantizedContiguousNodes.getEscapeIndex(leftChildNodexIndex);
		int leftSubTreeSizeInBytes = leftSubTreeSize * QuantizedBvhNodes.getNodeSize();

		//btQuantizedBvhNode& rightChildNode = m_quantizedContiguousNodes[rightChildNodexIndex];
		int rightSubTreeSize = _quantizedContiguousNodes.isLeafNode(rightChildNodexIndex) ? 1 : _quantizedContiguousNodes.getEscapeIndex(rightChildNodexIndex);
		int rightSubTreeSizeInBytes = rightSubTreeSize * QuantizedBvhNodes.getNodeSize();

		if (leftSubTreeSizeInBytes <= maxSubtreeSizeInBytes) {
			BvhSubtreeInfo subtree = BvhSubtreeInfo();
			SubtreeHeaders.add(subtree);

			subtree.setAabbFromQuantizeNode(_quantizedContiguousNodes, leftChildNodexIndex);
			subtree.rootNodeIndex = leftChildNodexIndex;
			subtree.subtreeSize = leftSubTreeSize;
		}

		if (rightSubTreeSizeInBytes <= maxSubtreeSizeInBytes) {
			BvhSubtreeInfo subtree = BvhSubtreeInfo();
			SubtreeHeaders.add(subtree);

			subtree.setAabbFromQuantizeNode(_quantizedContiguousNodes, rightChildNodexIndex);
			subtree.rootNodeIndex = rightChildNodexIndex;
			subtree.subtreeSize = rightSubTreeSize;
		}

		// PCK: update the copy of the size
		subtreeHeaderCount = SubtreeHeaders.size;
	}
	
	int sortAndCalcSplittingIndex(int startIndex, int endIndex, int splitAxis) {
		int i;
		int splitIndex = startIndex;
		int numIndices = endIndex - startIndex;
		double splitValue;

		Vector3 means = Vector3.zero();
		means.setValues(0, 0, 0);
		Vector3 center = Vector3.zero();
		for (i = startIndex; i < endIndex; i++) {
			center.add2(getAabbMax(i), getAabbMin(i));
			center.scale(0.5);
			means.add(center);
		}
		means.scale(1 / numIndices);

		splitValue = VectorUtil.getCoord(means, splitAxis);

		//sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
		for (i = startIndex; i < endIndex; i++) {
			//Vector3 center = Vector3.zero();
			center.add2(getAabbMax(i), getAabbMin(i));
			center.scale(0.5);

			if (VectorUtil.getCoord(center, splitAxis) > splitValue) {
				// swap
				swapLeafNodes(i, splitIndex);
				splitIndex++;
			}
		}

		// if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
		// otherwise the tree-building might fail due to stack-overflows in certain cases.
		// unbalanced1 is unsafe: it can cause stack overflows
		// bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

		// unbalanced2 should work too: always use center (perfect balanced trees)	
		// bool unbalanced2 = true;

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

	int calcSplittingAxis(int startIndex, int endIndex) {
		int i;

		Vector3 means = Vector3.zero();
		means.setValues(0, 0, 0);
		Vector3 variance = Vector3.zero();
		variance.setValues(0, 0, 0);
		int numIndices = endIndex - startIndex;

		Vector3 center = Vector3.zero();
		for (i = startIndex; i < endIndex; i++) {
			center.add2(getAabbMax(i), getAabbMin(i));
			center.scale(0.5);
			means.add(center);
		}
		means.scale(1 / numIndices);

		Vector3 diff2 = Vector3.zero();
		for (i = startIndex; i < endIndex; i++) {
			center.add2(getAabbMax(i), getAabbMin(i));
			center.scale(0.5);
			diff2.sub2(center, means);
			//diff2 = diff2 * diff2;
			VectorUtil.mul(diff2, diff2, diff2);
			variance.add(diff2);
		}
		variance.scale(1 / (numIndices - 1));

		return VectorUtil.maxAxis(variance);
	}

	void reportAabbOverlappingNodex(NodeOverlapCallback nodeCallback, Vector3 aabbMin, Vector3 aabbMax) {
		// either choose recursive traversal (walkTree) or stackless (walkStacklessTree)

		if (_useQuantization) {
			// quantize query AABB
			int quantizedQueryAabbMin;
			int quantizedQueryAabbMax;
			quantizedQueryAabbMin = quantizeWithClamp(aabbMin);
			quantizedQueryAabbMax = quantizeWithClamp(aabbMax);

			// JAVA TODO:
			switch (traversalMode) {
				case TraversalMode.stackless:
					walkStacklessQuantizedTree(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax, 0, _curNodeIndex);
					break;
					
//				case STACKLESS_CACHE_FRIENDLY:
//					walkStacklessQuantizedTreeCacheFriendly(nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);
//					break;
					
				case TraversalMode.recursive:
					walkRecursiveQuantizedTreeAgainstQueryAabb(_quantizedContiguousNodes, 0, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);
					break;
					
				default:
					assert (false); // unsupported
			}
		}
		else {
			walkStacklessTree(nodeCallback, aabbMin, aabbMax);
		}
	}
	
	void walkStacklessTree(NodeOverlapCallback nodeCallback, Vector3 aabbMin, Vector3 aabbMax) {
		assert (!_useQuantization);

		// JAVA NOTE: rewritten
		OptimizedBvhNode? rootNode;//contiguousNodes.get(0);
		int rootNode_index = 0;

		int escapeIndex, curIndex = 0;
		int walkIterations = 0;
		bool isLeafNode;
		//PCK: unsigned instead of bool
		//unsigned aabbOverlap;
		bool aabbOverlap;

		while (curIndex < _curNodeIndex) {
			// catch bugs in tree data
			assert (walkIterations < _curNodeIndex);

			walkIterations++;

			rootNode = _contiguousNodes.getQuick(rootNode_index);

			aabbOverlap = AabbUtil2.testAabbAgainstAabb2(aabbMin, aabbMax, rootNode!.aabbMinOrg, rootNode.aabbMaxOrg);
			isLeafNode = (rootNode.escapeIndex == -1);

			// PCK: unsigned instead of bool
			if (isLeafNode && (aabbOverlap/* != 0*/)) {
				nodeCallback.processNode(rootNode.subPart, rootNode.triangleIndex);
			}

			rootNode = null;

			//PCK: unsigned instead of bool
			if ((aabbOverlap/* != 0*/) || isLeafNode) {
				rootNode_index++;
				curIndex++;
			}
			else {
				escapeIndex = /*rootNode*/ _contiguousNodes.getQuick(rootNode_index)?.escapeIndex ?? 0;
				rootNode_index += escapeIndex;
				curIndex += escapeIndex;
			}
		}
		if (_maxIterations < walkIterations) {
			_maxIterations = walkIterations;
		}
	}

	void walkRecursiveQuantizedTreeAgainstQueryAabb(QuantizedBvhNodes currentNodes, int currentNodeId, NodeOverlapCallback nodeCallback, int quantizedQueryAabbMin, int quantizedQueryAabbMax) {
		assert (_useQuantization);

		bool isLeafNode;
		bool aabbOverlap;

		aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax, currentNodes.getQuantizedAabbMin(currentNodeId), currentNodes.getQuantizedAabbMax(currentNodeId));
		isLeafNode = currentNodes.isLeafNode(currentNodeId);

		if (aabbOverlap) {
			if (isLeafNode) {
				nodeCallback.processNode(currentNodes.getPartId(currentNodeId), currentNodes.getTriangleIndex(currentNodeId));
			}
			else {
				// process left and right children
				int leftChildNodeId = currentNodeId + 1;
				walkRecursiveQuantizedTreeAgainstQueryAabb(currentNodes, leftChildNodeId, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);

				int rightChildNodeId = currentNodes.isLeafNode(leftChildNodeId) ? leftChildNodeId + 1 : leftChildNodeId + currentNodes.getEscapeIndex(leftChildNodeId);
				walkRecursiveQuantizedTreeAgainstQueryAabb(currentNodes, rightChildNodeId, nodeCallback, quantizedQueryAabbMin, quantizedQueryAabbMax);
			}
		}
	}

	void walkStacklessQuantizedTreeAgainstRay(NodeOverlapCallback nodeCallback, Vector3 raySource, Vector3 rayTarget, Vector3 aabbMin, Vector3 aabbMax, int startNodeIndex, int endNodeIndex) {
		assert (_useQuantization);

		Vector3 tmp = Vector3.zero();

		int curIndex = startNodeIndex;
		int walkIterations = 0;
		int subTreeSize = endNodeIndex - startNodeIndex;

		QuantizedBvhNodes rootNode = _quantizedContiguousNodes;
		int rootNode_idx = startNodeIndex;
		int escapeIndex;

		bool isLeafNode;
		bool boxBoxOverlap = false;
		bool rayBoxOverlap = false;

		//double lambdaMax = 1;
		//#define RAYAABB2
		//#ifdef RAYAABB2
		//Vector3 rayFrom = Vector3.copy(raySource);
		Vector3 rayDirection = Vector3.zero();
		tmp.sub2(rayTarget, raySource);
		rayDirection.normalizeFrom(tmp);
		//lambdaMax = rayDirection.dot(tmp);
		rayDirection.x = 1 / rayDirection.x;
		rayDirection.y = 1 / rayDirection.y;
		rayDirection.z = 1/ rayDirection.z;
//		bool sign_x = rayDirection.x < 0;
//		bool sign_y = rayDirection.y < 0;
//		bool sign_z = rayDirection.z < 0;
		//#endif

		/* Quick pruning by quantized box */
		Vector3 rayAabbMin = Vector3.copy(raySource);
		Vector3 rayAabbMax = Vector3.copy(raySource);
		VectorUtil.setMin(rayAabbMin, rayTarget);
		VectorUtil.setMax(rayAabbMax, rayTarget);

		/* Add box cast extents to bounding box */
		rayAabbMin.add(aabbMin);
		rayAabbMax.add(aabbMax);

		int quantizedQueryAabbMin;
		int quantizedQueryAabbMax;
		quantizedQueryAabbMin = quantizeWithClamp(rayAabbMin);
		quantizedQueryAabbMax = quantizeWithClamp(rayAabbMax);

		Vector3 bounds_0 = Vector3.zero();
		Vector3 bounds_1 = Vector3.zero();
		Vector3 normal = Vector3.zero();
		double param = 0;

		while (curIndex < endNodeIndex) {
			assert (walkIterations < subTreeSize);

			walkIterations++;
			// only interested if this is closer than any previous hit
			param = 1;
			rayBoxOverlap = false;
			boxBoxOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax, rootNode.getQuantizedAabbMin(rootNode_idx), rootNode.getQuantizedAabbMax(rootNode_idx));
			isLeafNode = rootNode.isLeafNode(rootNode_idx);
			if (boxBoxOverlap) {
				unQuantize(bounds_0, rootNode.getQuantizedAabbMin(rootNode_idx));
				unQuantize(bounds_1, rootNode.getQuantizedAabbMax(rootNode_idx));
				bounds_0.add(aabbMin);
				bounds_1.add(aabbMax);
				rayBoxOverlap = AabbUtil2.rayAabb(raySource, rayTarget, bounds_0, bounds_1, param, normal);
			}

			if (isLeafNode && rayBoxOverlap) {
				nodeCallback.processNode(rootNode.getPartId(rootNode_idx), rootNode.getTriangleIndex(rootNode_idx));
			}

			if (rayBoxOverlap || isLeafNode) {
				rootNode_idx++;
				curIndex++;
			}
			else {
				escapeIndex = rootNode.getEscapeIndex(rootNode_idx);
				rootNode_idx += escapeIndex;
				curIndex += escapeIndex;
			}
		}
		
		if (_maxIterations < walkIterations) {
			_maxIterations = walkIterations;
		}
	}

	void walkStacklessQuantizedTree(NodeOverlapCallback nodeCallback, int quantizedQueryAabbMin, int quantizedQueryAabbMax, int startNodeIndex, int endNodeIndex) {
		assert (_useQuantization);

		int curIndex = startNodeIndex;
		int walkIterations = 0;
		int subTreeSize = endNodeIndex - startNodeIndex;

		QuantizedBvhNodes rootNode = _quantizedContiguousNodes;
		int rootNode_idx = startNodeIndex;
		int escapeIndex;

		bool isLeafNode;
		bool aabbOverlap;

		while (curIndex < endNodeIndex) {
			// catch bugs in tree data
			assert (walkIterations < subTreeSize);

			walkIterations++;
			aabbOverlap = testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin, quantizedQueryAabbMax, rootNode.getQuantizedAabbMin(rootNode_idx), rootNode.getQuantizedAabbMax(rootNode_idx));
			isLeafNode = rootNode.isLeafNode(rootNode_idx);

			if (isLeafNode && aabbOverlap) {
				nodeCallback.processNode(rootNode.getPartId(rootNode_idx), rootNode.getTriangleIndex(rootNode_idx));
			}

			if (aabbOverlap || isLeafNode) {
				rootNode_idx++;
				curIndex++;
			}
			else {
				escapeIndex = rootNode.getEscapeIndex(rootNode_idx);
				rootNode_idx += escapeIndex;
				curIndex += escapeIndex;
			}
		}

		if (_maxIterations < walkIterations) {
			_maxIterations = walkIterations;
		}
	}
	
	void reportRayOverlappingNodex(NodeOverlapCallback nodeCallback, Vector3 raySource, Vector3 rayTarget) {
		bool fastPath = _useQuantization && traversalMode == TraversalMode.stackless;
		if (fastPath) {
			Vector3 tmp = Vector3.zero();
			walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, tmp, tmp, 0, _curNodeIndex);
		}
		else {
			/* Otherwise fallback to AABB overlap test */
			Vector3 aabbMin = Vector3.copy(raySource);
			Vector3 aabbMax = Vector3.copy(raySource);
			VectorUtil.setMin(aabbMin, rayTarget);
			VectorUtil.setMax(aabbMax, rayTarget);
			reportAabbOverlappingNodex(nodeCallback, aabbMin, aabbMax);
		}
	}

	void reportBoxCastOverlappingNodex(NodeOverlapCallback nodeCallback, Vector3 raySource, Vector3 rayTarget, Vector3 aabbMin, Vector3 aabbMax) {
		bool fastPath = _useQuantization && traversalMode == TraversalMode.stackless;
		if (fastPath) {
			walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, _curNodeIndex);
		}
		else {
			/* Slow path:
			Construct the bounding box for the entire box cast and send that down the tree */
			Vector3 qaabbMin = Vector3.copy(raySource);
			Vector3 qaabbMax = Vector3.copy(raySource);
			VectorUtil.setMin(qaabbMin, rayTarget);
			VectorUtil.setMax(qaabbMax, rayTarget);
			qaabbMin.add(aabbMin);
			qaabbMax.add(aabbMax);
			reportAabbOverlappingNodex(nodeCallback, qaabbMin, qaabbMax);
		}
	}
	
	int quantizeWithClamp(Vector3 point) {
		assert (_useQuantization);

		Vector3 clampedPoint = Vector3.copy(point);
		VectorUtil.setMax(clampedPoint,_bvhAabbMin);
		VectorUtil.setMin(clampedPoint, _bvhAabbMax);

		Vector3 v = Vector3.zero();
		v.sub2(clampedPoint, _bvhAabbMin);
		VectorUtil.mul(v, v, _bvhQuantization);

		int out0 = (v.x + 0.5).toInt() & 0xFFFF;
		int out1 = (v.y + 0.5).toInt() & 0xFFFF;
		int out2 = (v.z + 0.5).toInt() & 0xFFFF;

		return (out0) | ((out1) << 16) | ((out2) << 32);
	}
	
	void unQuantize(Vector3 vecOut, int vecIn) {
		int vecIn0 = ((vecIn & 0x00000000FFFF));
		int vecIn1 = ((vecIn & 0x0000FFFF0000) >>> 16);
		int vecIn2 = ((vecIn & 0xFFFF00000000) >>> 32);

		vecOut.x = vecIn0 / (_bvhQuantization.x);
		vecOut.y = vecIn1 / (_bvhQuantization.y);
		vecOut.z = vecIn2 / (_bvhQuantization.z);

		vecOut.add(_bvhAabbMin);
	}
}

class _NodeTriangleCallback extends InternalTriangleIndexCallback {
  ObjectArrayList<OptimizedBvhNode> triangleNodes;
  
  _NodeTriangleCallback(this.triangleNodes);

  final Vector3 _aabbMin = Vector3.zero(), _aabbMax = Vector3.zero();
  
  @override
  void internalProcessTriangleIndex(List<Vector3> triangle, int partId, int triangleIndex) {
    OptimizedBvhNode node = OptimizedBvhNode();
    _aabbMin.setValues(1e30, 1e30, 1e30);
    _aabbMax.setValues(-1e30, -1e30, -1e30);
    VectorUtil.setMin(_aabbMin, triangle[0]);
    VectorUtil.setMax(_aabbMax, triangle[0]);
    VectorUtil.setMin(_aabbMin, triangle[1]);
    VectorUtil.setMax(_aabbMax, triangle[1]);
    VectorUtil.setMin(_aabbMin, triangle[2]);
    VectorUtil.setMax(_aabbMax, triangle[2]);

    // with quantization?
    node.aabbMinOrg.setFrom(_aabbMin);
    node.aabbMaxOrg.setFrom(_aabbMax);

    node.escapeIndex = -1;

    // for child nodes
    node.subPart = partId;
    node.triangleIndex = triangleIndex;
    triangleNodes.add(node);
  }
}

class _QuantizedNodeTriangleCallback extends InternalTriangleIndexCallback {
  //final BulletStack stack = BulletStack.get();
  QuantizedBvhNodes triangleNodes;
  OptimizedBvh optimizedTree; // for quantization

  _QuantizedNodeTriangleCallback(this.triangleNodes, this.optimizedTree);
  
  @override
  void internalProcessTriangleIndex(List<Vector3> triangle, int partId, int triangleIndex) {
    // The partId and triangle index must fit in the same (positive) integer
    assert (partId < (1 << OptimizedBvh.maxNumPartsInBits));
    assert (triangleIndex < (1 << (31 - OptimizedBvh.maxNumPartsInBits)));
    // negative indices are reserved for escapeIndex
    assert (triangleIndex >= 0);

    int nodeId = triangleNodes.add();
    Vector3 aabbMin = Vector3.zero(), aabbMax = Vector3.zero();
    aabbMin.setValues(1e30, 1e30, 1e30);
    aabbMax.setValues(-1e30, -1e30, -1e30);
    VectorUtil.setMin(aabbMin, triangle[0]);
    VectorUtil.setMax(aabbMax, triangle[0]);
    VectorUtil.setMin(aabbMin, triangle[1]);
    VectorUtil.setMax(aabbMax, triangle[1]);
    VectorUtil.setMin(aabbMin, triangle[2]);
    VectorUtil.setMax(aabbMax, triangle[2]);

    // PCK: add these checks for zero dimensions of aabb
    const double minAabbDimensions = 0.002;
    const double minAabbHalfDimensions = 0.001;
    
    if (aabbMax.x - aabbMin.x < minAabbDimensions) {
      aabbMax.x = (aabbMax.x + minAabbHalfDimensions);
      aabbMin.x = (aabbMin.x - minAabbHalfDimensions);
    }
    if (aabbMax.y - aabbMin.y < minAabbDimensions) {
      aabbMax.y = (aabbMax.y + minAabbHalfDimensions);
      aabbMin.y = (aabbMin.y - minAabbHalfDimensions);
    }
    if (aabbMax.z - aabbMin.z < minAabbDimensions) {
      aabbMax.z = (aabbMax.z + minAabbHalfDimensions);
      aabbMin.z = (aabbMin.z - minAabbHalfDimensions);
    }

    triangleNodes.setQuantizedAabbMin(nodeId, optimizedTree.quantizeWithClamp(aabbMin));
    triangleNodes.setQuantizedAabbMax(nodeId, optimizedTree.quantizeWithClamp(aabbMax));

    triangleNodes.setEscapeIndexOrTriangleIndex(nodeId, (partId << (31 - OptimizedBvh.maxNumPartsInBits)) | triangleIndex);
  }
}