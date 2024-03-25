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

import "package:bullet_physics/extras/gimpact/box_collision.dart";
import "package:bullet_physics/extras/gimpact/bvh_tree_node_array.dart";
import "package:bullet_physics/extras/gimpact/bvh_data_array.dart";
import "package:bullet_physics/extras/gimpact/bvh_tree.dart";
import "package:bullet_physics/extras/gimpact/pair_set.dart";
import "package:bullet_physics/extras/gimpact/primitive_manager_base.dart";
import "package:bullet_physics/extras/gimpact/primitive_triangle.dart";
import "package:bullet_physics/linearmath/aabb.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/utils/int_array_list.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class GImpactBvh {
	BvhTree boxTree = BvhTree();
	PrimitiveManagerBase? primitiveManager;

	/**
	 * This constructor doesn't build the tree. you must call buildSet.
	 */
	GImpactBvh([this.primitiveManager]);

	AABB getGlobalBox(AABB out) {
		getNodeBound(0, out);
		return out;
	}

	void setPrimitiveManager(PrimitiveManagerBase primitiveManager) {
		this.primitiveManager = primitiveManager;
	}

	PrimitiveManagerBase? getPrimitiveManager() {
		return primitiveManager;
	}
	
	// stackless refit
	void refit() {
		AABB leafbox = AABB();
		AABB bound = AABB();
		AABB tempBox = AABB();

		int nodecount = getNodeCount();
		while ((nodecount--) != 0) {
			if (isLeafNode(nodecount)) {
				primitiveManager?.getPrimitiveBox(getNodeData(nodecount), leafbox);
				setNodeBound(nodecount, leafbox);
			}
			else {
				//const BT_BVH_TREE_NODE * nodepointer = get_node_pointer(nodecount);
				//get left bound
				bound.invalidate();

				int childNode = getLeftNode(nodecount);
				if (childNode != 0) {
					getNodeBound(childNode, tempBox);
					bound.merge(tempBox);
				}

				childNode = getRightNode(nodecount);
				if (childNode != 0) {
					getNodeBound(childNode, tempBox);
					bound.merge(tempBox);
				}

				setNodeBound(nodecount, bound);
			}
		}
	}

	/**
	 * This attemps to refit the box set.
	 */
	void update(){
		refit();
	}

	/**
	 * This rebuild the entire set.
	 */
	void buildSet() {
		// obtain primitive boxes
		BvhDataArray primitiveBoxes = BvhDataArray();
		primitiveBoxes.resize(primitiveManager?.getPrimitiveCount());

		AABB tmpAABB = AABB();

		for (int i = 0; i < primitiveBoxes.length; i++) {
			//primitive_manager.get_primitive_box(i,primitiveBoxes[i].bound);
			primitiveManager?.getPrimitiveBox(i, tmpAABB);
			primitiveBoxes.setBound(i, tmpAABB);

			primitiveBoxes.setData(i, i);
		}

		boxTree.buildTree(primitiveBoxes);
	}

	/**
	 * Returns the indices of the primitives in the primitive_manager field.
	 */
	bool boxQuery(AABB box, IntArrayList collidedResults) {
		int curIndex = 0;
		int numNodes = getNodeCount();

		AABB bound = AABB();

		while (curIndex < numNodes) {
			getNodeBound(curIndex, bound);

			// catch bugs in tree data

			bool aabbOverlap = bound.hasCollision(box);
			bool isleafnode = isLeafNode(curIndex);

			if (isleafnode && aabbOverlap) {
				collidedResults.add(getNodeData(curIndex));
			}

			if (aabbOverlap || isleafnode) {
				// next subnode
				curIndex++;
			}
			else {
				// skip node
				curIndex += getEscapeNodeIndex(curIndex);
			}
		}
		if (collidedResults.size() > 0) {
			return true;
		}
		return false;
	}

	/**
	 * Returns the indices of the primitives in the primitive_manager field.
	 */
	bool boxQueryTrans(AABB box, Transform transform, IntArrayList collidedResults) {
		AABB transbox = AABB.fromAABB(box);
		transbox.appyTransform(transform);
		return boxQuery(transbox, collidedResults);
	}

	/**
	 * Returns the indices of the primitives in the primitive_manager field.
	 */
	bool rayQuery(Vector3 rayDir, Vector3 rayOrigin, IntArrayList collidedResults) {
		int curIndex = 0;
		int numNodes = getNodeCount();

		AABB bound = AABB();

		while (curIndex < numNodes) {
			getNodeBound(curIndex, bound);

			// catch bugs in tree data

			bool aabbOverlap = bound.collideRay(rayOrigin, rayDir);
			bool isleafnode = isLeafNode(curIndex);

			if (isleafnode && aabbOverlap) {
				collidedResults.add(getNodeData(curIndex));
			}

			if (aabbOverlap || isleafnode) {
				// next subnode
				curIndex++;
			}
			else {
				// skip node
				curIndex += getEscapeNodeIndex(curIndex);
			}
		}
		if (collidedResults.isNotEmpty) {
			return true;
		}
		return false;
	}

	/**
	 * Tells if this set has hierarchy.
	 */
	bool hasHierarchy() {
		return true;
	}
	
	/**
	 * Tells if this set is a trimesh.
	 */
	bool isTrimesh() {
		return primitiveManager?.isTrimesh() ?? false;
	}
	
	int getNodeCount() {
		return boxTree.getNodeCount();
	}
	
	/**
	 * Tells if the node is a leaf.
	 */
	bool isLeafNode(int nodeindex) {
		return boxTree.isLeafNode(nodeindex);
	}

	int getNodeData(int nodeindex) {
		return boxTree.getNodeData(nodeindex);
	}

	void getNodeBound(int nodeindex, AABB bound) {
		boxTree.getNodeBound(nodeindex, bound);
	}

	void setNodeBound(int nodeindex, AABB bound) {
		boxTree.setNodeBound(nodeindex, bound);
	}

	int getLeftNode(int nodeindex) {
		return boxTree.getLeftNode(nodeindex);
	}

	int getRightNode(int nodeindex) {
		return boxTree.getRightNode(nodeindex);
	}

	int getEscapeNodeIndex(int nodeindex) {
		return boxTree.getEscapeNodeIndex(nodeindex);
	}

	void getNodeTriangle(int nodeindex, PrimitiveTriangle triangle) {
		primitiveManager?.getPrimitiveTriangle(getNodeData(nodeindex), triangle);
	}

	BvhTreeNodeArray getNodePointer(){
		return boxTree.getNodePointer();
	}

	static bool _nodeCollision(GImpactBvh boxset0, GImpactBvh boxset1, BoxBoxTransformCache transCache1to0, int node0, int node1, bool completePrimitiveTests) {
		AABB box0 = AABB();
		boxset0.getNodeBound(node0, box0);
		AABB box1 = AABB();
		boxset1.getNodeBound(node1, box1);

		return box0.overlappingTransCache(box1, transCache1to0, completePrimitiveTests);
		//box1.appy_transform_trans_cache(transCache1to0);
		//return box0.has_collision(box1);
	}
	
	/**
	 * Stackless recursive collision routine.
	 */
	static void _findCollisionPairsRecursive(GImpactBvh boxset0, GImpactBvh boxset1, PairSet collisionPairs, BoxBoxTransformCache transCache1to0, int node0, int node1, bool completePrimitiveTests) {
		if (_nodeCollision(
				boxset0, boxset1, transCache1to0,
				node0, node1, completePrimitiveTests) == false) {
			return;//avoid colliding internal nodes
		}
		if (boxset0.isLeafNode(node0)) {
			if (boxset1.isLeafNode(node1)) {
				// collision result
				collisionPairs.pushPair(boxset0.getNodeData(node0), boxset1.getNodeData(node1));
				return;
			}
			else {
				// collide left recursive
				_findCollisionPairsRecursive(
						boxset0, boxset1,
						collisionPairs, transCache1to0,
						node0, boxset1.getLeftNode(node1), false);

				// collide right recursive
				_findCollisionPairsRecursive(
						boxset0, boxset1,
						collisionPairs, transCache1to0,
						node0, boxset1.getRightNode(node1), false);
			}
		}
		else {
			if (boxset1.isLeafNode(node1)) {
				// collide left recursive
				_findCollisionPairsRecursive(
						boxset0, boxset1,
						collisionPairs, transCache1to0,
						boxset0.getLeftNode(node0), node1, false);


				// collide right recursive
				_findCollisionPairsRecursive(
						boxset0, boxset1,
						collisionPairs, transCache1to0,
						boxset0.getRightNode(node0), node1, false);
			}
			else {
				// collide left0 left1
				_findCollisionPairsRecursive(
						boxset0, boxset1,
						collisionPairs, transCache1to0,
						boxset0.getLeftNode(node0), boxset1.getLeftNode(node1), false);

				// collide left0 right1
				_findCollisionPairsRecursive(
						boxset0, boxset1,
						collisionPairs, transCache1to0,
						boxset0.getLeftNode(node0), boxset1.getRightNode(node1), false);

				// collide right0 left1
				_findCollisionPairsRecursive(
						boxset0, boxset1,
						collisionPairs, transCache1to0,
						boxset0.getRightNode(node0), boxset1.getLeftNode(node1), false);

				// collide right0 right1
				_findCollisionPairsRecursive(
						boxset0, boxset1,
						collisionPairs, transCache1to0,
						boxset0.getRightNode(node0), boxset1.getRightNode(node1), false);

			} // else if node1 is not a leaf
		} // else if node0 is not a leaf
	}
	
	//static double getAverageTreeCollisionTime();

	static void findCollision(GImpactBvh boxset0, Transform? trans0, GImpactBvh boxset1, Transform? trans1, PairSet collisionPairs) {
		if (boxset0.getNodeCount() == 0 || boxset1.getNodeCount() == 0) {
			return;
		}
		BoxBoxTransformCache transCache1to0 = BoxBoxTransformCache();

		transCache1to0.calcFromHomogenic(trans0, trans1);

		//#ifdef TRI_COLLISION_PROFILING
		//bt_begin_gim02_tree_time();
		//#endif //TRI_COLLISION_PROFILING

		_findCollisionPairsRecursive(
				boxset0, boxset1,
				collisionPairs, transCache1to0, 0, 0, true);

		//#ifdef TRI_COLLISION_PROFILING
		//bt_end_gim02_tree_time();
		//#endif //TRI_COLLISION_PROFILING
	}
	
}
