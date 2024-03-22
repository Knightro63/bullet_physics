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

import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm_construction_info.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/collision_algorithm_create_func.dart";
import "package:bullet_physics/collision/dispatch/collision_dispatcher.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/compound_shape.dart";
import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/static_plane_shape.dart";
import "package:bullet_physics/extras/gimpact/g_impact_bvh.dart";
import "package:bullet_physics/extras/gimpact/g_impact_mesh_shape.dart";
import "package:bullet_physics/extras/gimpact/g_impact_mesh_shape_part.dart";
import "package:bullet_physics/extras/gimpact/g_impact_shape_interface.dart";
import "package:bullet_physics/extras/gimpact/g_impact_triangle_callback.dart";
import "package:bullet_physics/extras/gimpact/gim_shape_retriever.dart";
import "package:bullet_physics/extras/gimpact/pair.dart";
import "package:bullet_physics/extras/gimpact/pair_set.dart";
import "package:bullet_physics/extras/gimpact/plane_intersection_type.dart";
import "package:bullet_physics/extras/gimpact/plane_shape.dart";
import "package:bullet_physics/extras/gimpact/primitive_triangle.dart";
import "package:bullet_physics/extras/gimpact/shape_type.dart";
import "package:bullet_physics/extras/gimpact/triangle_contact.dart";
import "package:bullet_physics/linearmath/aabb.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import "package:bullet_physics/utils/int_array_list.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

/**
 * Collision Algorithm for GImpact Shapes.<p>
 * 
 * For register this algorithm in Bullet, proceed as following:
 * <pre>
 * CollisionDispatcher dispatcher = (CollisionDispatcher)dynamicsWorld.getDispatcher();
 * GImpactCollisionAlgorithm.registerAlgorithm(dispatcher);
 * </pre>
 * 
 * @author jezek2
 */
class GImpactCollisionAlgorithm extends CollisionAlgorithm {

	CollisionAlgorithm? convexAlgorithm;
  PersistentManifold? manifoldPtr;
	ManifoldResult? resultOut;
	DispatcherInfo? dispatchInfo;
	int triface0 = 0;
	int part0 = 0;
	int triface1 = 0;
	int part1 = 0;

	final PairSet _tmpPairset = PairSet();
	
	void init(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
		super.initCA(ci);
		manifoldPtr = null;
		convexAlgorithm = null;
	}
	
	@override
	void destroy() {
		clearCache();
	}

	@override
	void processCollision(CollisionObject? body0, CollisionObject? body1, DispatcherInfo? dispatchInfo, ManifoldResult? resultOut) {
		clearCache();

		this.resultOut = resultOut;
		this.dispatchInfo = dispatchInfo;
		GImpactShapeInterface? gimpactshape0;
		GImpactShapeInterface? gimpactshape1;

		if (body0?.getCollisionShape()?.getShapeType() == BroadphaseNativeType.gimpactShapeProxytype){
			gimpactshape0 = body0?.getCollisionShape() as GImpactShapeInterface?;

			if( body1?.getCollisionShape()?.getShapeType() == BroadphaseNativeType.gimpactShapeProxytype ){
				gimpactshape1 = body1?.getCollisionShape() as GImpactShapeInterface?;

				gimpactVSgimpact(body0,body1,gimpactshape0,gimpactshape1);
			}
			else{
				gimpactVSshape(body0,body1,gimpactshape0,body1?.getCollisionShape(),false);
			}

		}
		else if (body1?.getCollisionShape()?.getShapeType() == BroadphaseNativeType.gimpactShapeProxytype )
		{
			gimpactshape1 = body1?.getCollisionShape() as GImpactShapeInterface?;

			gimpactVSshape(body1,body0,gimpactshape1,body0?.getCollisionShape(),true);
		}
	}
	
	void gimpactVSgimpact(CollisionObject? body0, CollisionObject? body1, GImpactShapeInterface? shape0, GImpactShapeInterface? shape1) {
		if (shape0?.getGImpactShapeType() == ShapeType.trimeshShape) {
			GImpactMeshShape? meshshape0 = shape0 as GImpactMeshShape?;
			part0 = meshshape0?.getMeshPartCount() ?? 0;

			while ((part0--) != 0) {
				gimpactVSgimpact(body0, body1, meshshape0?.getMeshPart(part0), shape1);
			}

			return;
		}

		if (shape1?.getGImpactShapeType() == ShapeType.trimeshShape) {
			GImpactMeshShape? meshshape1 = shape1 as GImpactMeshShape?;
			part1 = meshshape1?.getMeshPartCount() ?? 0;

			while ((part1--) != 0) {
				gimpactVSgimpact(body0, body1, shape0, meshshape1?.getMeshPart(part1));
			}

			return;
		}

		Transform? orgtrans0 = body0?.getWorldTransform(Transform());
		Transform? orgtrans1 = body1?.getWorldTransform(Transform());

		PairSet pairset = _tmpPairset;
		pairset.clear();

		gimpactVSgimpactFindPairs(orgtrans0, orgtrans1, shape0, shape1, pairset);

		if (pairset.length == 0) {
			return;
		}
		if (shape0?.getGImpactShapeType() == ShapeType.trimeshShapePart &&
		    shape1?.getGImpactShapeType() == ShapeType.trimeshShapePart) {
			
			GImpactMeshShapePart? shapepart0 = shape0 as GImpactMeshShapePart?;
			GImpactMeshShapePart? shapepart1 = shape1 as GImpactMeshShapePart?;
			
			//specialized function
			//#ifdef BULLET_TRIANGLE_COLLISION
			//collide_gjk_triangles(body0,body1,shapepart0,shapepart1,&pairset[0].m_index1,pairset.length);
			//#else
			collideSatTriangles(body0, body1, shapepart0, shapepart1, pairset, pairset.length);
			//#endif

			return;
		}

		// general function

		shape0?.lockChildShapes();
		shape1?.lockChildShapes();

		GIM_ShapeRetriever retriever0 = GIM_ShapeRetriever(shape0);
		GIM_ShapeRetriever retriever1 = GIM_ShapeRetriever(shape1);

		bool childHasTransform0 = shape0?.childrenHasTransform() ?? false;
		bool childHasTransform1 = shape1?.childrenHasTransform() ?? false;

		Transform tmpTrans = Transform();

		int i = pairset.length;
		while ((i--) != 0) {
			Pair pair = pairset.get(i);
			triface0 = pair.index1;
			triface1 = pair.index2;
			CollisionShape? colshape0 = retriever0.getChildShape(triface0);
			CollisionShape? colshape1 = retriever1.getChildShape(triface1);

			if (childHasTransform0) {
				tmpTrans.mul(orgtrans0, shape0?.getChildTransform(triface0));
				body0?.setWorldTransform(tmpTrans);
			}

			if (childHasTransform1) {
				tmpTrans.mul(orgtrans1, shape1?.getChildTransform(triface1));
				body1?.setWorldTransform(tmpTrans);
			}

			// collide two convex shapes
			convexVSconvexCollision(body0, body1, colshape0, colshape1);

			if (childHasTransform0) {
				body0?.setWorldTransform(orgtrans0);
			}

			if (childHasTransform1) {
				body1?.setWorldTransform(orgtrans1);
			}

		}

		shape0?.unlockChildShapes();
		shape1?.unlockChildShapes();
	}
	
	void gimpactVSshape(CollisionObject? body0, CollisionObject? body1, GImpactShapeInterface? shape0, CollisionShape? shape1, bool swapped) {
		if (shape0?.getGImpactShapeType() == ShapeType.trimeshShape) {
			GImpactMeshShape? meshshape0 = shape0 as GImpactMeshShape?;
			part0 = meshshape0?.getMeshPartCount() ?? 0;

			while ((part0--) != 0) {
				gimpactVSshape(
          body0,
					body1,
					meshshape0?.getMeshPart(part0),
					shape1, 
          swapped
        );
			}

			return;
		}

		//#ifdef GIMPACT_VS_PLANE_COLLISION
		if (shape0?.getGImpactShapeType() == ShapeType.trimeshShapePart &&
				shape1?.getShapeType() == BroadphaseNativeType.staticPlaneProxytype) {
			GImpactMeshShapePart shapepart = shape0 as GImpactMeshShapePart;
			StaticPlaneShape planeshape = shape1 as StaticPlaneShape;
			gimpactTrimeshPartVSplaneCollision(body0, body1, shapepart, planeshape, swapped);
			return;
		}
		//#endif

		if (shape1?.isCompound() ?? false) {
			CompoundShape compoundshape = shape1 as CompoundShape;
			gimpactVScompoundshape(body0, body1, shape0, compoundshape, swapped);
			return;
		}
		else if (shape1?.isConcave() ?? false) {
			ConcaveShape concaveshape = shape1 as ConcaveShape;
			gimpactVSconcave(body0, body1, shape0, concaveshape, swapped);
			return;
		}

		Transform? orgtrans0 = body0?.getWorldTransform(Transform());
		Transform? orgtrans1 = body1?.getWorldTransform(Transform());

		IntArrayList collidedResults = IntArrayList();

		gimpactVSshapeFindPairs(orgtrans0, orgtrans1, shape0, shape1, collidedResults);

		if (collidedResults.isEmpty) {
			return;
		}
		shape0?.lockChildShapes();

		GIM_ShapeRetriever retriever0 = GIM_ShapeRetriever(shape0);

		bool childHasTransform0 = shape0?.childrenHasTransform() ?? false;

		Transform tmpTrans = Transform();

		int i = collidedResults.size();

		while ((i--) != 0) {
			int childIndex = collidedResults[i];
			if (swapped) {
				triface1 = childIndex;
			}
			else {
				triface0 = childIndex;
			}
			CollisionShape? colshape0 = retriever0.getChildShape(childIndex);

			if (childHasTransform0) {
				tmpTrans.mul(orgtrans0, shape0?.getChildTransform(childIndex));
				body0?.setWorldTransform(tmpTrans);
			}

			// collide two shapes
			if (swapped) {
				shapeVSshapeCollision(body1, body0, shape1, colshape0);
			}
			else {
				shapeVSshapeCollision(body0, body1, colshape0, shape1);
			}

			// restore transforms
			if (childHasTransform0) {
				body0?.setWorldTransform(orgtrans0);
			}

		}

		shape0?.unlockChildShapes();
	}
	
	void gimpactVScompoundshape(CollisionObject? body0, CollisionObject? body1, GImpactShapeInterface? shape0, CompoundShape shape1, bool swapped) {
		Transform? orgtrans1 = body1?.getWorldTransform(Transform());
		Transform childtrans1 = Transform();
		Transform tmpTrans = Transform();

		int i = shape1.getNumChildShapes();
		while ((i--) != 0) {
			CollisionShape? colshape1 = shape1.getChildShape(i);
			childtrans1.mul(orgtrans1, shape1.getChildTransform(i, tmpTrans));

			body1?.setWorldTransform(childtrans1);

			// collide child shape
			gimpactVSshape(body0, body1,
					shape0, colshape1, swapped);

			// restore transforms
			body1?.setWorldTransform(orgtrans1);
		}
	}
	
	void gimpactVSconcave(CollisionObject? body0, CollisionObject? body1, GImpactShapeInterface? shape0, ConcaveShape shape1, bool swapped) {
		// create the callback
		GImpactTriangleCallback tricallback = GImpactTriangleCallback();
		tricallback.algorithm = this;
		tricallback.body0 = body0;
		tricallback.body1 = body1;
		tricallback.gimpactshape0 = shape0;
		tricallback.swapped = swapped;
		tricallback.margin = shape1.getMargin();

		// getting the trimesh AABB
		Transform gimpactInConcaveSpace = Transform();

		body1?.getWorldTransform(gimpactInConcaveSpace);
		gimpactInConcaveSpace.inverse();
		gimpactInConcaveSpace.mul(body0?.getWorldTransform(Transform()));

		Vector3 minAABB = Vector3.zero(), maxAABB = Vector3.zero();
		shape0?.getAabb(gimpactInConcaveSpace, minAABB, maxAABB);
		shape1.processAllTriangles(tricallback, minAABB, maxAABB);
	}
	
	/**
	 * Creates a contact point.
	 */
	PersistentManifold? newContactManifold(CollisionObject? body0, CollisionObject? body1) {
		manifoldPtr = dispatcher?.getNewManifold(body0, body1);
		return manifoldPtr;
	}

	void destroyConvexAlgorithm() {
		if (convexAlgorithm != null) {
			//convexAlgorithm.destroy();
			dispatcher?.freeCollisionAlgorithm(convexAlgorithm);
			convexAlgorithm = null;
		}
	}

	void destroyContactManifolds() {
		if (manifoldPtr == null) return;
		dispatcher?.releaseManifold(manifoldPtr);
		manifoldPtr = null;
	}

	void clearCache() {
		destroyContactManifolds();
		destroyConvexAlgorithm();

		triface0 = -1;
		part0 = -1;
		triface1 = -1;
		part1 = -1;
	}

	PersistentManifold? getLastManifold() {
		return manifoldPtr;
	}

	/**
	 * Call before process collision.
	 */
	void checkManifold(CollisionObject? body0, CollisionObject? body1) {
		if (getLastManifold() == null) {
			newContactManifold(body0, body1);
		}

		resultOut?.setPersistentManifold(getLastManifold());
	}
	
	/**
	 * Call before process collision.
	 */
	CollisionAlgorithm? newAlgorithm(CollisionObject? body0, CollisionObject? body1) {
		checkManifold(body0, body1);

		CollisionAlgorithm? convexAlgorithm = dispatcher?.findAlgorithm(body0, body1, getLastManifold());
		return convexAlgorithm;
	}
	
	/**
	 * Call before process collision.
	 */
	void checkConvexAlgorithm(CollisionObject? body0, CollisionObject? body1) {
		if (convexAlgorithm != null) return;
		convexAlgorithm = newAlgorithm(body0, body1);
	}

	void addContactPoint(CollisionObject? body0, CollisionObject? body1, Vector3 point, Vector3 normal, double distance) {
		resultOut?.setShapeIdentifiers(part0, triface0, part1, triface1);
		checkManifold(body0, body1);
		resultOut?.addContactPoint(normal, point, distance);
	}

	/*
	void collide_gjk_triangles(CollisionObject body0, CollisionObject body1, GImpactMeshShapePart shape0, GImpactMeshShapePart shape1, IntArrayList pairs, int pair_count) {
	}
	*/
	
	void collideSatTriangles(CollisionObject? body0, CollisionObject? body1, GImpactMeshShapePart? shape0, GImpactMeshShapePart? shape1, PairSet pairs, int pair_count) {
		Vector3 tmp = Vector3.zero();

		Transform? orgtrans0 = body0?.getWorldTransform(Transform());
		Transform? orgtrans1 = body1?.getWorldTransform(Transform());

		PrimitiveTriangle ptri0 = PrimitiveTriangle();
		PrimitiveTriangle ptri1 = PrimitiveTriangle();
		TriangleContact contactData = TriangleContact();

		shape0?.lockChildShapes();
		shape1?.lockChildShapes();

		int pairPointer = 0;

		while ((pair_count--) != 0) {
			//triface0 = pairs.get(pairPointer);
			//triface1 = pairs.get(pairPointer + 1);
			//pairPointer += 2;
			Pair pair = pairs.get(pairPointer++);
			triface0 = pair.index1;
			triface1 = pair.index2;

			shape0?.getPrimitiveTriangle(triface0, ptri0);
			shape1?.getPrimitiveTriangle(triface1, ptri1);

			//#ifdef TRI_COLLISION_PROFILING
			//bt_begin_gim02_tri_time();
			//#endif

			ptri0.applyTransform(orgtrans0);
			ptri1.applyTransform(orgtrans1);

			// build planes
			ptri0.buildTriPlane();
			ptri1.buildTriPlane();

			// test conservative
			if (ptri0.overlapTestConservative(ptri1)) {
				if (ptri0.findTriangleCollisionClipMethod(ptri1, contactData)) {

					int j = contactData.pointCount;
					while ((j--) != 0) {
						tmp.x = contactData.separatingNormal.x;
						tmp.y = contactData.separatingNormal.y;
						tmp.z = contactData.separatingNormal.z;

						addContactPoint(body0, body1,
								contactData.points[j],
								tmp,
								-contactData.penetrationDepth);
					}
				}
			}

			//#ifdef TRI_COLLISION_PROFILING
			//bt_end_gim02_tri_time();
			//#endif
		}

		shape0?.unlockChildShapes();
		shape1?.unlockChildShapes();
	}

	void shapeVSshapeCollision(CollisionObject? body0, CollisionObject? body1, CollisionShape? shape0, CollisionShape? shape1) {
		CollisionShape? tmpShape0 = body0?.getCollisionShape();
		CollisionShape? tmpShape1 = body1?.getCollisionShape();

		body0?.internalSetTemporaryCollisionShape(shape0);
		body1?.internalSetTemporaryCollisionShape(shape1);

		{
			CollisionAlgorithm? algor = newAlgorithm(body0, body1);
			// post :	checkManifold is called

			resultOut?.setShapeIdentifiers(part0, triface0, part1, triface1);

			algor?.processCollision(body0, body1, dispatchInfo, resultOut);

			//algor.destroy();
			dispatcher?.freeCollisionAlgorithm(algor);
		}

		body0?.internalSetTemporaryCollisionShape(tmpShape0);
		body1?.internalSetTemporaryCollisionShape(tmpShape1);
	}
	
	void convexVSconvexCollision(CollisionObject? body0, CollisionObject? body1, CollisionShape? shape0, CollisionShape? shape1) {
		CollisionShape? tmpShape0 = body0?.getCollisionShape();
		CollisionShape? tmpShape1 = body1?.getCollisionShape();

		body0?.internalSetTemporaryCollisionShape(shape0);
		body1?.internalSetTemporaryCollisionShape(shape1);

		resultOut?.setShapeIdentifiers(part0, triface0, part1, triface1);

		checkConvexAlgorithm(body0, body1);
		convexAlgorithm?.processCollision(body0, body1, dispatchInfo, resultOut);

		body0?.internalSetTemporaryCollisionShape(tmpShape0);
		body1?.internalSetTemporaryCollisionShape(tmpShape1);
	}

	void gimpactVSgimpactFindPairs(Transform? trans0, Transform? trans1, GImpactShapeInterface? shape0, GImpactShapeInterface? shape1, PairSet pairset) {
		if ((shape0?.hasBoxSet() ?? false) && (shape1?.hasBoxSet() ?? false)) {
			GImpactBvh.findCollision(shape0!.getBoxSet(), trans0, shape1!.getBoxSet(), trans1, pairset);
		}
		else {
			AABB boxshape0 = AABB();
			AABB boxshape1 = AABB();
			int i = shape0?.getNumChildShapes() ?? 0;

			while ((i--) != 0) {
				shape0?.getChildAabb(i, trans0, boxshape0.min, boxshape0.max);

				int j = shape1?.getNumChildShapes() ?? 0;
				while ((j--) != 0) {
					shape1?.getChildAabb(i, trans1, boxshape1.min, boxshape1.max);

					if (boxshape1.hasCollision(boxshape0)) {
						pairset.pushPair(i, j);
					}
				}
			}
		}
	}

	void gimpactVSshapeFindPairs(Transform? trans0, Transform? trans1, GImpactShapeInterface? shape0, CollisionShape? shape1, IntArrayList collidedPrimitives) {
		AABB boxshape = AABB();

		if (shape0?.hasBoxSet()??false) {
			Transform trans1to0 = Transform();
			trans1to0.inverse(trans0);
			trans1to0.mul(trans1);

			shape1?.getAabb(trans1to0, boxshape.min, boxshape.max);

			shape0?.getBoxSet().boxQuery(boxshape, collidedPrimitives);
		}
		else {
			shape1?.getAabb(trans1!, boxshape.min, boxshape.max);

			AABB boxshape0 = AABB();
			int i = shape0?.getNumChildShapes() ?? 0;

			while ((i--) != 0) {
				shape0?.getChildAabb(i, trans0, boxshape0.min, boxshape0.max);

				if (boxshape.hasCollision(boxshape0)) {
					collidedPrimitives.add(i);
				}
			}
		}
	}
	
	void gimpactTrimeshPartVSplaneCollision(CollisionObject? body0, CollisionObject? body1, GImpactMeshShapePart shape0, StaticPlaneShape shape1, bool swapped) {
		Transform? orgtrans0 = body0?.getWorldTransform(Transform());
		Transform? orgtrans1 = body1?.getWorldTransform(Transform());

		StaticPlaneShape planeshape = shape1;
		Vector4 plane = Vector4.zero();
		PlaneShape.getPlaneEquationTransformed(planeshape, orgtrans1, plane);

		// test box against plane

		AABB tribox = AABB();
		shape0.getAabb(orgtrans0, tribox.min, tribox.max);
		tribox.incrementMargin(planeshape.getMargin());

		if (tribox.plane_classify(plane) != PlaneIntersectionType.collidePlane) {
			return;
		}
		shape0.lockChildShapes();

		double margin = shape0.getMargin() + planeshape.getMargin();

		Vector3 vertex = Vector3.zero();

		Vector3 tmp = Vector3.zero();

		int vi = shape0.getVertexCount();
		while ((vi--) != 0) {
			shape0.getVertex(vi, vertex);
			orgtrans0?.transform(vertex);

			double distance = VectorUtil.dot3(vertex, plane) - plane.w - margin;

			if (distance < 0){
				if (swapped) {
					tmp.setValues(-plane.x, -plane.y, -plane.z);
					addContactPoint(body1, body0, vertex, tmp, distance);
				}
				else {
					tmp.setValues(plane.x, plane.y, plane.z);
					addContactPoint(body0, body1, vertex, tmp, distance);
				}
			}
		}

		shape0.unlockChildShapes();
	}
	
	
	void setFace0(int value) {
		triface0 = value;
	}

	int getFace0() {
		return triface0;
	}

	void setFace1(int value) {
		triface1 = value;
	}

	int getFace1() {
		return triface1;
	}

	void setPart0(int value) {
		part0 = value;
	}

	int getPart0() {
		return part0;
	}

	void setPart1(int value) {
		part1 = value;
	}

	int getPart1() {
		return part1;
	}

	@override
	double calculateTimeOfImpact(CollisionObject? body0, CollisionObject? body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
		return 1;
	}

	@override
	void getAllContactManifolds(ObjectArrayList<PersistentManifold> manifoldArray) {
		if (manifoldPtr != null) {
			manifoldArray.add(manifoldPtr!);
		}
	}
	
	////////////////////////////////////////////////////////////////////////////
	/**
	 * Use this function for register the algorithm externally.
	 */
	static void registerAlgorithm(CollisionDispatcher dispatcher) {
		GCAFunc createFunc = GCAFunc();

		for (int i=0; i<BroadphaseNativeType.maxBroadphaseCollisionTypes.index; i++) {
			dispatcher.registerCollisionCreateFunc(BroadphaseNativeType.gimpactShapeProxytype.index, i, createFunc);
		}

		for (int i=0; i<BroadphaseNativeType.maxBroadphaseCollisionTypes.index; i++) {
			dispatcher.registerCollisionCreateFunc(i, BroadphaseNativeType.gimpactShapeProxytype.index, createFunc);
		}
	}	
}

class GCAFunc extends CollisionAlgorithmCreateFunc {
  @override
  CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject? body0, CollisionObject? body1) {
    GImpactCollisionAlgorithm algo =GImpactCollisionAlgorithm();
    algo.init(ci, body0, body1);
    return algo;
  }

  @override
  void releaseCollisionAlgorithm(CollisionAlgorithm algo) {

  }
}
