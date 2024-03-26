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

import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/dispatch/collision_world.dart";
import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/extras/gimpact/g_impact_bvh.dart";
import "package:bullet_physics/extras/gimpact/primitive_manager_base.dart";
import "package:bullet_physics/extras/gimpact/primitive_triangle.dart";
import "package:bullet_physics/extras/gimpact/shape_type.dart";
import "package:bullet_physics/extras/gimpact/tetrahedron_shape_ex.dart";
import "package:bullet_physics/extras/gimpact/triangle_shape_ex.dart";
import "package:bullet_physics/linearmath/aabb.dart";
import "package:bullet_physics/linearmath/transform.dart";
import 'package:vector_math/vector_math.dart';

abstract class GImpactShapeInterface extends ConcaveShape {
  AABB localAABB = AABB();
  bool needsUpdate = false;
  final Vector3 localScaling = Vector3.zero();
  GImpactBvh boxSet = GImpactBvh(); // optionally boxset

	GImpactShapeInterface() {
		localAABB.invalidate();
		needsUpdate = true;
		localScaling.setValues(1, 1, 1);
	}

	/**
	 * Performs refit operation.<p>
	 * Updates the entire Box set of this shape.<p>
	 * 
	 * postUpdate() must be called for attemps to calculating the box set, else this function
	 * will does nothing.<p>
	 * 
	 * if m_needs_update == true, then it calls calcLocalAABB();
	 */
	void updateBound() {
		if (!needsUpdate) {
			return;
		}
		calcLocalAABB();
		needsUpdate = false;
	}

	/**
	 * If the Bounding box is not updated, then this class attemps to calculate it.<p>
     * Calls updateBound() for update the box set.
     */
	@override
	void getAabb(Transform? t, Vector3 aabbMin, Vector3 aabbMax) {
		AABB transformedbox = AABB.fromAABB(localAABB);
		transformedbox.appyTransform(t);
		aabbMin.setFrom(transformedbox.min);
		aabbMax.setFrom(transformedbox.max);
	}

	/**
	 * Tells to this object that is needed to refit the box set.
	 */
	void postUpdate() {
		needsUpdate = true;
	}
	
	/**
	 * Obtains the local box, which is the global calculated box of the total of subshapes.
	 */
	AABB getLocalBox(AABB out) {
		out.set(localAABB);
		return out;
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.gimpactShapeProxytype;
	}

	/**
	 * You must call updateBound() for update the box set.
	 */
	@override
	void setLocalScaling(Vector3 scaling) {
		localScaling.setFrom(scaling);
		postUpdate();
	}

	@override
	Vector3 getLocalScaling(Vector3 out) {
		out.setFrom(localScaling);
		return out;
	}

	@override
	void setMargin(double margin) {
		collisionMargin = margin;
		int i = getNumChildShapes();
		while ((i--) != 0) {
			CollisionShape? child = getChildShape(i);
			child?.setMargin(margin);
		}

		needsUpdate = true;
	}

	/**
	 * Base method for determinig which kind of GIMPACT shape we get.
	 */
	ShapeType getGImpactShapeType();
	
	GImpactBvh getBoxSet() {
		return boxSet;
	}

	/**
	 * Determines if this class has a hierarchy structure for sorting its primitives.
	 */
	bool hasBoxSet() {
		if (boxSet.getNodeCount() == 0) {
			return false;
		}
		return true;
	}

	/**
	 * Obtains the primitive manager.
	 */
	PrimitiveManagerBase? getPrimitiveManager();

	/**
	 * Gets the number of children.
	 */
	int getNumChildShapes();

	/**
	 * If true, then its children must get transforms.
	 */
	bool childrenHasTransform();

	/**
	 * Determines if this shape has triangles.
	 */
	bool needsRetrieveTriangles();

	/**
	 * Determines if this shape has tetrahedrons.
	 */
	bool needsRetrieveTetrahedrons();

	void getBulletTriangle(int primIndex, TriangleShapeEx? triangle);

	void getBulletTetrahedron(int primIndex, TetrahedronShapeEx? tetrahedron);

	/**
	 * Call when reading child shapes.
	 */
	void lockChildShapes() {
	}

	void unlockChildShapes() {
	}
	
	/**
	 * If this trimesh.
	 */
	void getPrimitiveTriangle(int index, PrimitiveTriangle triangle) {
		getPrimitiveManager()?.getPrimitiveTriangle(index, triangle);
	}
	
	/**
	 * Use this function for perfofm refit in bounding boxes.
	 */
	void calcLocalAABB() {
		lockChildShapes();
		if (boxSet.getNodeCount() == 0) {
			boxSet.buildSet();
		}
		else {
			boxSet.update();
		}
		unlockChildShapes();

		boxSet.getGlobalBox(localAABB);
	}
	
	/**
	 * Retrieves the bound from a child.
	 */
	void getChildAabb(int childIndex, Transform? t, Vector3 aabbMin, Vector3 aabbMax) {
		AABB childAABB = AABB();
		getPrimitiveManager()?.getPrimitiveBox(childIndex, childAABB);
		childAABB.appyTransform(t);
		aabbMin.setFrom(childAABB.min);
		aabbMax.setFrom(childAABB.max);
	}

	/**
	 * Gets the children.
	 */
	CollisionShape? getChildShape(int index);
	
	/**
	 * Gets the children transform.
	 */
	Transform? getChildTransform(int index);

	/**
	 * Sets the children transform.<p>
	 * You must call updateBound() for update the box set.
	 */
	void setChildTransform(int index, Transform transform);

	/**
	 * Virtual method for ray collision.
	 */
	void rayTest(Vector3 rayFrom, Vector3 rayTo, RayResultCallback resultCallback) {}
	
	/**
	 * Function for retrieve triangles. It gives the triangles in local space.
	 */
	@override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax) {}
}
