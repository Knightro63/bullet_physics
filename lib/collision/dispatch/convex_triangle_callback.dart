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

import "package:bullet_physics/collision/broadphase/collision_algorithm.dart";
import "package:bullet_physics/collision/broadphase/collision_algorithm_construction_info.dart";
import "package:bullet_physics/collision/broadphase/dispatcher.dart";
import "package:bullet_physics/collision/broadphase/dispatcher_info.dart";
import "package:bullet_physics/collision/dispatch/collision_object.dart";
import "package:bullet_physics/collision/dispatch/manifold_result.dart";
import "package:bullet_physics/collision/narrowphase/persistent_manifold.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/collision/shapes/triangle_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import 'package:vector_math/vector_math.dart';

/**
 * For each triangle in the concave mesh that overlaps with the AABB of a convex
 * (see {@link #convexBody} field), processTriangle is called.
 * 
 * @author jezek2
 */
class ConvexTriangleCallback extends TriangleCallback {

	//final BulletStack stack = BulletStack.get();
	
	CollisionObject? _convexBody;
	CollisionObject? _triBody;

	final Vector3 _aabbMin = Vector3.zero();
	final Vector3 _aabbMax = Vector3.zero();

	ManifoldResult? _resultOut;

	Dispatcher? _dispatcher;
	DispatcherInfo? _dispatchInfoPtr;
	double _collisionMarginTriangle = 0;
	
	int triangleCount = 0;
	PersistentManifold? manifoldPtr;
	
	ConvexTriangleCallback(Dispatcher? dispatcher, CollisionObject? body0, CollisionObject? body1, bool isSwapped) {
		_dispatcher = dispatcher;
		_convexBody = isSwapped ? body1 : body0;
		_triBody = isSwapped ? body0 : body1;
		//
		// create the manifold from the dispatcher 'manifold pool'
		//
		manifoldPtr = dispatcher?.getNewManifold(_convexBody, _triBody);
		clearCache();
	}
	
	void destroy() {
		clearCache();
		_dispatcher?.releaseManifold(manifoldPtr);
	}

	void setTimeStepAndCounters(double collisionMarginTriangle, DispatcherInfo? dispatchInfo, ManifoldResult? resultOut) {
		_dispatchInfoPtr = dispatchInfo;
		_collisionMarginTriangle = collisionMarginTriangle;
		_resultOut = resultOut;

		// recalc aabbs
		Transform convexInTriangleSpace = Transform();

		_triBody?.getWorldTransform(convexInTriangleSpace);
		convexInTriangleSpace.inverse();
		convexInTriangleSpace.mul(_convexBody?.getWorldTransform(Transform()));

		CollisionShape? convexShape = _convexBody?.getCollisionShape();
		//CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
		convexShape?.getAabb(convexInTriangleSpace, _aabbMin, _aabbMax);
		double extraMargin = collisionMarginTriangle;
		Vector3 extra = Vector3.zero();
		extra.setValues(extraMargin, extraMargin, extraMargin);

		_aabbMax.add(extra);
		_aabbMin.sub(extra);
	}

	CollisionAlgorithmConstructionInfo _ci = CollisionAlgorithmConstructionInfo();
	TriangleShape _tm = TriangleShape();
	
  @override
	void processTriangle(List<Vector3> triangle, int partId, int triangleIndex) {
		// just for debugging purposes
		//printf("triangle %d",m_triangleCount++);

		// aabb filter is already applied!	

		_ci.dispatcher1 = _dispatcher;

		CollisionObject? ob = _triBody;

		// debug drawing of the overlapping triangles
		if (_dispatchInfoPtr != null && _dispatchInfoPtr!.debugDraw != null && _dispatchInfoPtr!.debugDraw!.getDebugMode() > 0) {
			Vector3 color = Vector3.zero();
			color.setValues(255, 255, 0);
			Transform? tr = ob?.getWorldTransform(Transform());

			Vector3 tmp1 = Vector3.zero();
			Vector3 tmp2 = Vector3.zero();

			tmp1.setFrom(triangle[0]); 
      tr?.transform(tmp1);
			tmp2.setFrom(triangle[1]); 
      tr?.transform(tmp2);
			_dispatchInfoPtr!.debugDraw!.drawLine(tmp1, tmp2, color);

			tmp1.setFrom(triangle[1]); 
      tr?.transform(tmp1);
			tmp2.setFrom(triangle[2]); 
      tr?.transform(tmp2);
			_dispatchInfoPtr!.debugDraw!.drawLine(tmp1, tmp2, color);

			tmp1.setFrom(triangle[2]); 
      tr?.transform(tmp1);
			tmp2.setFrom(triangle[0]); 
      tr?.transform(tmp2);
			_dispatchInfoPtr!.debugDraw!.drawLine(tmp1, tmp2, color);

			//btVector3 center = triangle[0] + triangle[1]+triangle[2];
			//center *= btScalar(0.333333);
			//m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(center),color);
			//m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(center),color);
			//m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(center),color);
		}

		//btCollisionObject* colObj = static_cast<btCollisionObject*>(m_convexProxy->m_clientObject);

		if (_convexBody?.getCollisionShape() != null && _convexBody!.getCollisionShape()!.isConvex()) {
			_tm.init(triangle[0], triangle[1], triangle[2]);
			_tm.setMargin(_collisionMarginTriangle);

			CollisionShape? tmpShape = ob?.getCollisionShape();
			ob?.internalSetTemporaryCollisionShape(_tm);

			CollisionAlgorithm? colAlgo = _ci.dispatcher1?.findAlgorithm(_convexBody, _triBody, manifoldPtr);
			// this should use the btDispatcher, so the actual registered algorithm is used
			//		btConvexConvexAlgorithm cvxcvxalgo(m_manifoldPtr,ci,m_convexBody,m_triBody);

			_resultOut?.setShapeIdentifiers(-1, -1, partId, triangleIndex);
			//cvxcvxalgo.setShapeIdentifiers(-1,-1,partId,triangleIndex);
			//cvxcvxalgo.processCollision(m_convexBody,m_triBody,*m_dispatchInfoPtr,m_resultOut);
			colAlgo?.processCollision(_convexBody, _triBody, _dispatchInfoPtr, _resultOut);
			//colAlgo.destroy();
			_ci.dispatcher1?.freeCollisionAlgorithm(colAlgo);
			ob?.internalSetTemporaryCollisionShape(tmpShape);
		}
	}

	void clearCache() {
		_dispatcher?.clearManifold(manifoldPtr);
	}

	Vector3 getAabbMin(Vector3 out) {
		out.setFrom(_aabbMin);
		return out;
	}

	Vector3 getAabbMax(Vector3 out) {
		out.setFrom(_aabbMax);
		return out;
	}
	
}
