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

import "package:bullet_physics/collision/dispatch/collision_world.dart";
import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/striding_mesh_interface.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/extras/gimpact/g_impact_mesh_shape_part.dart";
import "package:bullet_physics/extras/gimpact/g_impact_shape_interface.dart";
import "package:bullet_physics/extras/gimpact/primitive_manager_base.dart";
import "package:bullet_physics/extras/gimpact/shape_type.dart";
import "package:bullet_physics/extras/gimpact/tetrahedron_shape_ex.dart";
import "package:bullet_physics/extras/gimpact/triangle_shape_ex.dart";
import "package:bullet_physics/linearmath/aabb.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/utils/object_array_list.dart";
import 'package:vector_math/vector_math.dart';

/**
 *
 * @author jezek2
 */
class GImpactMeshShape extends GImpactShapeInterface {
	
	ObjectArrayList<GImpactMeshShapePart> meshParts = ObjectArrayList();//List<GImpactMeshShapePart>();

	GImpactMeshShape(StridingMeshInterface meshInterface) {
		buildMeshParts(meshInterface);
	}
	
	int getMeshPartCount() {
		return meshParts.size;
	}

	GImpactMeshShapePart? getMeshPart(int index) {
		return meshParts.getQuick(index);
	}

	@override
	void setLocalScaling(Vector3 scaling) {
		localScaling.setFrom(scaling);

		int i = meshParts.size;
		while ((i--) != 0) {
			GImpactMeshShapePart? part = meshParts.getQuick(i);
			part?.setLocalScaling(scaling);
		}

		needsUpdate = true;
	}

	@override
	void setMargin(double margin) {
		collisionMargin = margin;

		int i = meshParts.size;
		while ((i--) != 0) {
			GImpactMeshShapePart? part = meshParts.getQuick(i);
			part?.setMargin(margin);
		}

		needsUpdate = true;
	}

	@override
	void postUpdate() {
		int i = meshParts.size;
		while ((i--) != 0) {
			GImpactMeshShapePart? part = meshParts.getQuick(i);
			part?.postUpdate();
		}

		needsUpdate = true;
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		//#ifdef CALC_EXACT_INERTIA
		inertia.setValues(0, 0, 0);

		int i = getMeshPartCount();
		double partmass = mass / i;

		Vector3 partinertia = Vector3.zero();

		while ((i--) != 0) {
			getMeshPart(i)?.calculateLocalInertia(partmass, partinertia);
			inertia.add(partinertia);
		}

		////#else
		//
		//// Calc box inertia
		//
		//btScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
		//btScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
		//btScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
		//const btScalar x2 = lx*lx;
		//const btScalar y2 = ly*ly;
		//const btScalar z2 = lz*lz;
		//const btScalar scaledmass = mass * btScalar(0.08333333);
		//
		//inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));
		////#endif
	}
	
	@override
	PrimitiveManagerBase? getPrimitiveManager() {
		assert (false);
		return null;
	}

	@override
	int getNumChildShapes() {
		assert (false);
		return 0;
	}

	@override
	bool childrenHasTransform() {
		assert (false);
		return false;
	}

	@override
	bool needsRetrieveTriangles() {
		assert (false);
		return false;
	}

	@override
	bool needsRetrieveTetrahedrons() {
		assert (false);
		return false;
	}

	@override
	void getBulletTriangle(int primIndex, TriangleShapeEx? triangle) {
		assert (false);
	}

	@override
	void getBulletTetrahedron(int primIndex, TetrahedronShapeEx? tetrahedron) {
		assert (false);
	}

	@override
	void lockChildShapes() {
		assert (false);
	}

	@override
	void unlockChildShapes() {
		assert (false);
	}

	@override
	void getChildAabb(int childIndex, Transform? t, Vector3 aabbMin, Vector3 aabbMax) {
		assert (false);
	}

	@override
	CollisionShape? getChildShape(int index) {
		assert (false);
		return null;
	}

	@override
	Transform? getChildTransform(int index) {
		assert (false);
		return null;
	}

	@override
	void setChildTransform(int index, Transform transform) {
		assert (false);
	}

	@override
	ShapeType getGImpactShapeType() {
		return ShapeType.trimeshShape;
	}

	@override
	String getName() {
		return "GImpactMesh";
	}

	@override
	void rayTest(Vector3 rayFrom, Vector3 rayTo, RayResultCallback resultCallback) {
	}

	@override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax) {
		int i = meshParts.size;
		while ((i--) != 0) {
			meshParts.getQuick(i)?.processAllTriangles(callback, aabbMin, aabbMax);
		}
	}
	
	void buildMeshParts(StridingMeshInterface meshInterface) {
		for (int i=0; i<meshInterface.getNumSubParts(); i++) {
			GImpactMeshShapePart newpart = GImpactMeshShapePart(meshInterface, i);
			meshParts.add(newpart);
		}
	}

	@override
	void calcLocalAABB() {
		AABB tmpAABB = AABB();

		localAABB.invalidate();
		int i = meshParts.size;
		while ((i--) != 0) {
			meshParts.getQuick(i)?.updateBound();
			localAABB.merge(meshParts.getQuick(i)!.getLocalBox(tmpAABB));
		}
	}

}
