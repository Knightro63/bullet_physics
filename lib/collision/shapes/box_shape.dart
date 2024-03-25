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

import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/shapes/polyhedral_convex_shape.dart";
import "package:bullet_physics/linearmath/aabb_util2.dart";
import "package:bullet_physics/linearmath/scalar_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class BoxShape extends PolyhedralConvexShape {

	BoxShape(Vector3 boxHalfExtents) {
		Vector3 margin = Vector3(getMargin(), getMargin(), getMargin());
		VectorUtil.mul(implicitShapeDimensions, boxHalfExtents, localScaling);
		implicitShapeDimensions.sub(margin);
	}

	Vector3 getHalfExtentsWithMargin(Vector3 out) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(out);
		Vector3 margin = Vector3.zero();
		margin.setValues(getMargin(), getMargin(), getMargin());
		halfExtents.add(margin);
		return out;
	}

	Vector3 getHalfExtentsWithoutMargin(Vector3 out) {
		out.setFrom(implicitShapeDimensions); // changed in Bullet 2.63: assume the scaling and margin are included
		return out;
	}

  @override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.boxShapeProxyType;
	}
  @override
	Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(out);
		
		double margin = getMargin();
		halfExtents.x += margin;
		halfExtents.y += margin;
		halfExtents.z += margin;

		out.setValues(
				ScalarUtil.fsel(vec.x, halfExtents.x, -halfExtents.x),
				ScalarUtil.fsel(vec.y, halfExtents.y, -halfExtents.y),
				ScalarUtil.fsel(vec.z, halfExtents.z, -halfExtents.z));
		return out;
	}

	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(out);

		out.setValues(
				ScalarUtil.fsel(vec.x, halfExtents.x, -halfExtents.x),
				ScalarUtil.fsel(vec.y, halfExtents.y, -halfExtents.y),
				ScalarUtil.fsel(vec.z, halfExtents.z, -halfExtents.z));
		return out;
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(Vector3.zero());

		for (int i = 0; i < numVectors; i++) {
			Vector3 vec = vectors[i];
			supportVerticesOut[i].setValues(ScalarUtil.fsel(vec.x, halfExtents.x, -halfExtents.x),
					ScalarUtil.fsel(vec.y, halfExtents.y, -halfExtents.y),
					ScalarUtil.fsel(vec.z, halfExtents.z, -halfExtents.z));
		}
	}

  @override
	void setMargin(double margin) {
		// correct the implicitShapeDimensions for the margin
		Vector3 oldMargin = Vector3.zero();
		oldMargin.setValues(getMargin(), getMargin(), getMargin());
		Vector3 implicitShapeDimensionsWithMargin = Vector3.zero();
		implicitShapeDimensionsWithMargin.add(implicitShapeDimensions-oldMargin);

		super.setMargin(margin);
		Vector3 newMargin = Vector3.zero();
		newMargin.setValues(getMargin(), getMargin(), getMargin());
		implicitShapeDimensions.sub(implicitShapeDimensionsWithMargin-newMargin);
	}

	@override
	void setLocalScaling(Vector3 scaling) {
		Vector3 oldMargin = Vector3.zero();
		oldMargin.setValues(getMargin(), getMargin(), getMargin());
		Vector3 implicitShapeDimensionsWithMargin = Vector3.zero();
		implicitShapeDimensionsWithMargin.add(implicitShapeDimensions-oldMargin);
		Vector3 unScaledImplicitShapeDimensionsWithMargin = Vector3.zero();
		VectorUtil.div(unScaledImplicitShapeDimensionsWithMargin, implicitShapeDimensionsWithMargin, localScaling);

		super.setLocalScaling(scaling);

		VectorUtil.mul(implicitShapeDimensions, unScaledImplicitShapeDimensionsWithMargin, localScaling);
		implicitShapeDimensions.sub(oldMargin);
	}

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		AabbUtil2.transformAabb(getHalfExtentsWithoutMargin(Vector3.zero()), getMargin(), t, aabbMin, aabbMax);
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		//btScalar margin = btScalar(0.);
		Vector3 halfExtents = getHalfExtentsWithMargin(Vector3.zero());

		double lx = 2 * halfExtents.x;
		double ly = 2 * halfExtents.y;
		double lz = 2 * halfExtents.z;

		inertia.setValues(mass / 12 * (ly * ly + lz * lz),
				mass / 12 * (lx * lx + lz * lz),
				mass / 12 * (lx * lx + ly * ly));
	}

	@override
	void getPlane(Vector3 planeNormal, Vector3 planeSupport, int i) {
		// this plane might not be aligned...
		Vector4 plane = Vector4.zero();
		getPlaneEquation(plane, i);
		planeNormal.setValues(plane.x, plane.y, plane.z);
		Vector3 tmp = Vector3.zero();
		tmp.negateFrom(planeNormal);
		localGetSupportingVertex(tmp, planeSupport);
	}

	@override
	int getNumPlanes() {
		return 6;
	}

	@override
	int getNumVertices() {
		return 8;
	}

	@override
	int getNumEdges() {
		return 12;
	}

	@override
	void getVertex(int i, Vector3 vtx) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(Vector3.zero());

		vtx.setValues(halfExtents.x * (1 - (i & 1)) - halfExtents.x * (i & 1),
				halfExtents.y * (1 - ((i & 2) >> 1)) - halfExtents.y * ((i & 2) >> 1),
				halfExtents.z * (1 - ((i & 4) >> 2)) - halfExtents.z * ((i & 4) >> 2));
	}
	
	void getPlaneEquation(Vector4 plane, int i) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(Vector3.zero());

		switch (i) {
			case 0:
				plane.setValues(1, 0, 0, -halfExtents.x);
				break;
			case 1:
				plane.setValues(-1, 0, 0, -halfExtents.x);
				break;
			case 2:
				plane.setValues(0, 1, 0, -halfExtents.y);
				break;
			case 3:
				plane.setValues(0, -1, 0, -halfExtents.y);
				break;
			case 4:
				plane.setValues(0, 0, 1, -halfExtents.z);
				break;
			case 5:
				plane.setValues(0, 0, -1, -halfExtents.z);
				break;
			default:
				assert (false);
		}
	}

	@override
	void getEdge(int i, Vector3 pa, Vector3 pb) {
		int edgeVert0 = 0;
		int edgeVert1 = 0;

		switch (i) {
			case 0:
				edgeVert0 = 0;
				edgeVert1 = 1;
				break;
			case 1:
				edgeVert0 = 0;
				edgeVert1 = 2;
				break;
			case 2:
				edgeVert0 = 1;
				edgeVert1 = 3;

				break;
			case 3:
				edgeVert0 = 2;
				edgeVert1 = 3;
				break;
			case 4:
				edgeVert0 = 0;
				edgeVert1 = 4;
				break;
			case 5:
				edgeVert0 = 1;
				edgeVert1 = 5;

				break;
			case 6:
				edgeVert0 = 2;
				edgeVert1 = 6;
				break;
			case 7:
				edgeVert0 = 3;
				edgeVert1 = 7;
				break;
			case 8:
				edgeVert0 = 4;
				edgeVert1 = 5;
				break;
			case 9:
				edgeVert0 = 4;
				edgeVert1 = 6;
				break;
			case 10:
				edgeVert0 = 5;
				edgeVert1 = 7;
				break;
			case 11:
				edgeVert0 = 6;
				edgeVert1 = 7;
				break;
			default:
				assert (false);
		}

		getVertex(edgeVert0, pa);
		getVertex(edgeVert1, pb);
	}

	@override
	bool isInside(Vector3 pt, double tolerance) {
		Vector3 halfExtents = getHalfExtentsWithoutMargin(Vector3.zero());

		//btScalar minDist = 2*tolerance;

		bool result =
				(pt.x <= (halfExtents.x + tolerance)) &&
				(pt.x >= (-halfExtents.x - tolerance)) &&
				(pt.y <= (halfExtents.y + tolerance)) &&
				(pt.y >= (-halfExtents.y - tolerance)) &&
				(pt.z <= (halfExtents.z + tolerance)) &&
				(pt.z >= (-halfExtents.z - tolerance));

		return result;
	}

	@override
	String getName() {
		return "Box";
	}

	@override
	int getNumPreferredPenetrationDirections() {
		return 6;
	}

	@override
	void getPreferredPenetrationDirection(int index, Vector3 penetrationVector) {
		switch (index) {
			case 0:
				penetrationVector.setValues(1, 0, 0);
				break;
			case 1:
				penetrationVector.setValues(-1, 0, 0);
				break;
			case 2:
				penetrationVector.setValues(0, 1, 0);
				break;
			case 3:
				penetrationVector.setValues(0, -1, 0);
				break;
			case 4:
				penetrationVector.setValues(0, 0, 1);
				break;
			case 5:
				penetrationVector.setValues(0, 0, -1);
				break;
			default:
				assert (false);
		}
	}

}
