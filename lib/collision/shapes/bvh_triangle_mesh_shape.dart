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

import "package:bullet_physics/collision/shapes/node_overlap_callback.dart";
import "package:bullet_physics/collision/shapes/optimized_bvh.dart";
import "package:bullet_physics/collision/shapes/striding_mesh_interface.dart";
import "package:bullet_physics/collision/shapes/triangle_mesh_shape.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/collision/shapes/vertex_data.dart";
import "package:bullet_physics/core/bullet_globals.dart";

import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/linearmath/vector_util.dart";

import 'package:vector_math/vector_math.dart';

class BvhTriangleMeshShape extends TriangleMeshShape {
	OptimizedBvh? _bvh;
	bool _useQuantizedAabbCompression = false;
	bool _ownsBvh = false;

	/**
	 * Optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb.
	 */
	BvhTriangleMeshShape([StridingMeshInterface? meshInterface, bool useQuantizedAabbCompression = false, Vector3? bvhAabbMin, Vector3? bvhAabbMax, bool buildBvh = true]):super(meshInterface) {
		if(bvhAabbMin == null || bvhAabbMax == null){
      bvhAabbMin ??= Vector3.zero();
      bvhAabbMax ??= Vector3.zero();
      meshInterface?.calculateAabbBruteForce(bvhAabbMin, bvhAabbMax);
    }

    _bvh = null;
		_useQuantizedAabbCompression = useQuantizedAabbCompression;
		_ownsBvh = false;
		if (buildBvh) {
			_bvh = OptimizedBvh();
			_bvh?.build(meshInterface, useQuantizedAabbCompression, bvhAabbMin, bvhAabbMax);
			_ownsBvh = true;
      recalcLocalAabb();
		}
	}

	bool getOwnsBvh() {
		return _ownsBvh;
	}
	
	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.triangleMeshShapeProxytype;
	}

	void performRaycast(TriangleCallback callback, Vector3 raySource, Vector3 rayTarget) {
		MyNodeOverlapCallback myNodeCallback = MyNodeOverlapCallback();
		myNodeCallback.init(callback, meshInterface);
		_bvh?.reportRayOverlappingNodex(myNodeCallback, raySource, rayTarget);
	}
	
	void performConvexcast(TriangleCallback callback, Vector3 raySource, Vector3 rayTarget, Vector3 aabbMin, Vector3 aabbMax) {
		MyNodeOverlapCallback myNodeCallback = MyNodeOverlapCallback();
		myNodeCallback.init(callback, meshInterface);
		_bvh?.reportBoxCastOverlappingNodex(myNodeCallback, raySource, rayTarget, aabbMin, aabbMax);
	}

	/**
	 * Perform bvh tree traversal and report overlapping triangles to 'callback'.
	 */
	@override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax) {
		//#ifdef DISABLE_BVH
		// // brute force traverse all triangles
		//btTriangleMeshShape::processAllTriangles(callback,aabbMin,aabbMax);
		//#else

		// first get all the nodes
		MyNodeOverlapCallback myNodeCallback = MyNodeOverlapCallback();
		myNodeCallback.init(callback, meshInterface);

		_bvh?.reportAabbOverlappingNodex(myNodeCallback, aabbMin, aabbMax);
		//#endif//DISABLE_BVH
	}
	
	void refitTree(Vector3 aabbMin, Vector3 aabbMax) {
		//bvh.refit(meshInterface, aabbMin, aabbMax);
		_bvh?.refit(meshInterface);
		recalcLocalAabb();
	}

	/**
	 * For a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks.
	 */
	void partialRefitTree(Vector3 aabbMin, Vector3 aabbMax) {
		_bvh?.refitPartial(meshInterface,aabbMin,aabbMax );
		VectorUtil.setMin(localAabbMin, aabbMin);
		VectorUtil.setMax(localAabbMax, aabbMax);
	}

	@override
	String getName() {
		return "BVHTRIANGLEMESH";
	}
	
	@override
	void setLocalScaling(Vector3 scaling) {
		Vector3 tmp = Vector3.zero();
		tmp.sub2(getLocalScaling(Vector3.zero()), scaling);

		if (tmp.length2 > BulletGlobals.simdEpsilon) {
			super.setLocalScaling(scaling);
			/*
			if (ownsBvh)
			{
			m_bvh->~btOptimizedBvh();
			btAlignedFree(m_bvh);
			}
			*/
			///m_localAabbMin/m_localAabbMax is already re-calculated in btTriangleMeshShape. We could just scale aabb, but this needs some more work
			_bvh = OptimizedBvh();
			// rebuild the bvh...
			_bvh?.build(meshInterface, _useQuantizedAabbCompression, localAabbMin, localAabbMax);
			_ownsBvh = true;
		}
	}
	
	OptimizedBvh? getOptimizedBvh() {
		return _bvh;
	}

	void setOptimizedBvh(OptimizedBvh bvh, [Vector3? scaling]) {
    scaling ??= Vector3(1,1,1);
		assert (_bvh == null);
		assert (!_ownsBvh);

		_bvh = bvh;
		_ownsBvh = false;

		// update the scaling without rebuilding the bvh
		Vector3 tmp = Vector3.zero();
		tmp.sub2(getLocalScaling(Vector3.zero()), scaling);

		if (tmp.length2 > BulletGlobals.simdEpsilon) {
			super.setLocalScaling(scaling);
		}
	}

	bool usesQuantizedAabbCompression() {
		return _useQuantizedAabbCompression;
	}	
}

////////////////////////////////////////////////////////////////////////////

class MyNodeOverlapCallback extends NodeOverlapCallback {
  StridingMeshInterface? meshInterface;
  TriangleCallback? callback;

  final List<Vector3> _triangle = [ Vector3.zero(), Vector3.zero(), Vector3.zero()];

  MyNodeOverlapCallback();
  
  void init(TriangleCallback? callback, StridingMeshInterface? meshInterface) {
    this.meshInterface = meshInterface;
    this.callback = callback;
  }
  @override
  void processNode(int nodeSubPart, int nodeTriangleIndex) {
    VertexData? data = meshInterface?.getLockedReadOnlyVertexIndexBase(nodeSubPart);

    Vector3 meshScaling = meshInterface?.getScaling(Vector3.zero()) ?? Vector3.zero();

    data?.getTriangle(nodeTriangleIndex*3, meshScaling, _triangle);

    /* Perform ray vs. triangle collision here */
    callback?.processTriangle(_triangle, nodeSubPart, nodeTriangleIndex);
    
    meshInterface?.unLockReadOnlyVertexBase(nodeSubPart);
  }
}