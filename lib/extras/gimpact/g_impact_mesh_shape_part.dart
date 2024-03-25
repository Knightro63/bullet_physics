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

import "package:bullet_physics/collision/shapes/collision_shape.dart";
import "package:bullet_physics/collision/shapes/striding_mesh_interface.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/extras/gimpact/g_impact_mass_util.dart";
import "package:bullet_physics/extras/gimpact/g_impact_shape_interface.dart";
import "package:bullet_physics/extras/gimpact/primitive_manager_base.dart";
import "package:bullet_physics/extras/gimpact/primitive_triangle.dart";
import "package:bullet_physics/extras/gimpact/shape_type.dart";
import "package:bullet_physics/extras/gimpact/tetrahedron_shape_ex.dart";
import "package:bullet_physics/extras/gimpact/triangle_shape_ex.dart";
import "package:bullet_physics/extras/gimpact/trimesh_primitive_manager.dart";
import "package:bullet_physics/linearmath/aabb.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/utils/int_array_list.dart";
import 'package:vector_math/vector_math.dart';

/**
 * This class manages a sub part of a mesh supplied by the StridingMeshInterface interface.<p>
 * 
 * - Simply create this shape by passing the StridingMeshInterface to the constructor
 *   GImpactMeshShapePart, then you must call updateBound() after creating the mesh<br>
 * - When making operations with this shape, you must call <b>lock</b> before accessing
 *   to the trimesh primitives, and then call <b>unlock</b><br>
 * - You can handle deformable meshes with this shape, by calling postUpdate() every time
 *   when changing the mesh vertices.
 * 
 * @author jezek2
 */
class GImpactMeshShapePart extends GImpactShapeInterface {

	TrimeshPrimitiveManager primitiveManager = TrimeshPrimitiveManager();
	
	final IntArrayList _collided = IntArrayList();
	

	GImpactMeshShapePart([StridingMeshInterface? meshInterface, int part = 0]) {
		primitiveManager.meshInterface = meshInterface;
		primitiveManager.part = part;
		boxSet.setPrimitiveManager(primitiveManager);
	}

	@override
	bool childrenHasTransform() {
		return false;
	}

	@override
	void lockChildShapes() {
		TrimeshPrimitiveManager? dummymanager = boxSet.getPrimitiveManager() as TrimeshPrimitiveManager?;
		dummymanager?.lock();
	}

	@override
	void unlockChildShapes() {
		TrimeshPrimitiveManager? dummymanager = boxSet.getPrimitiveManager() as TrimeshPrimitiveManager?;
		dummymanager?.unlock();
	}

	@override
	int getNumChildShapes() {
		return primitiveManager.getPrimitiveCount();
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
	PrimitiveManagerBase getPrimitiveManager() {
		return primitiveManager;
	}
	
	TrimeshPrimitiveManager getTrimeshPrimitiveManager() {
		return primitiveManager;
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		lockChildShapes();
		
		//#define CALC_EXACT_INERTIA 1
		//#ifdef CALC_EXACT_INERTIA
		inertia.setValues(0, 0, 0);

		int i = getVertexCount();
		double pointmass = mass / i;

		Vector3 pointintertia = Vector3.zero();

		while ((i--) != 0) {
			getVertex(i, pointintertia);
			GImpactMassUtil.getPointInertia(pointintertia, pointmass, pointintertia);
			inertia.add(pointintertia);
		}

		//#else
		//
		//// Calc box inertia
		//
		//double lx= localAABB.max.x - localAABB.min.x;
		//double ly= localAABB.max.y - localAABB.min.y;
		//double lz= localAABB.max.z - localAABB.min.z;
		//double x2 = lx*lx;
		//double y2 = ly*ly;
		//double z2 = lz*lz;
		//double scaledmass = mass * 0.08333333f;
		//
		//inertia.set(y2+z2,x2+z2,x2+y2);
		//inertia.scale(scaledmass);
		//
		//#endif
		unlockChildShapes();
	}

	@override
	String getName() {
		return "GImpactMeshShapePart";
	}
	
	@override
	ShapeType getGImpactShapeType() {
		return ShapeType.trimeshShapePart;
	}

	@override
	bool needsRetrieveTriangles() {
		return true;
	}

	@override
	bool needsRetrieveTetrahedrons() {
		return false;
	}

	@override
	void getBulletTriangle(int primIndex, TriangleShapeEx? triangle) {
		primitiveManager.getBulletTriangle(primIndex, triangle);
	}

	@override
	void getBulletTetrahedron( int primIndex, TetrahedronShapeEx? tetrahedron) {
		assert (false);
	}

	int getVertexCount() {
		return primitiveManager.getVertexCount();
	}

	void getVertex(int vertexIndex, Vector3 vertex) {
		primitiveManager.getVertex(vertexIndex, vertex);
	}

	@override
	void setMargin(double margin) {
		primitiveManager.margin = margin;
		postUpdate();
	}

	@override
	double getMargin() {
		return primitiveManager.margin;
	}

	@override
	void setLocalScaling(Vector3 scaling) {
		primitiveManager.scale.setFrom(scaling);
		postUpdate();
	}

	@override
	Vector3 getLocalScaling(Vector3 out) {
		out.setFrom(primitiveManager.scale);
		return out;
	}

	int getPart() {
		return primitiveManager.part;
	}

	@override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax) {
		lockChildShapes();
		AABB box = AABB();
		box.min.setFrom(aabbMin);
		box.max.setFrom(aabbMax);

		_collided.clear();
		boxSet.boxQuery(box, _collided);

		if (_collided.isEmpty) {
			unlockChildShapes();
			return;
		}

		int part = getPart();
		PrimitiveTriangle triangle = PrimitiveTriangle();
		int i = _collided.size();
		while ((i--) != 0) {
			getPrimitiveTriangle(_collided[i], triangle);
			callback?.processTriangle(triangle.vertices, part, _collided[i]);
		}
		unlockChildShapes();
	}
	
}
