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

import 'dart:math';
import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/shapes/convex_internal_shape.dart";
import "package:bullet_physics/core/bullet_globals.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class ConeShape extends ConvexInternalShape {
	late double _sinAngle;
	late double _radius;
	late double _height;
	List<int> _coneIndices = [0,0,0];

	ConeShape(double radius, double height) {
		_radius = radius;
		_height = height;
		setConeUpIndex(1);
		_sinAngle = (radius / sqrt(_radius * _radius + _height * _height));
	}

	double getRadius() {
		return _radius;
	}

	double getHeight() {
		return _height;
	}

	Vector3 _coneLocalSupport(Vector3 v, Vector3 out) {
		double halfHeight = _height * 0.5;

		if (VectorUtil.getCoord(v, _coneIndices[1]) > v.length * _sinAngle) {
			VectorUtil.setCoord(out, _coneIndices[0], 0);
			VectorUtil.setCoord(out, _coneIndices[1], halfHeight);
			VectorUtil.setCoord(out, _coneIndices[2], 0);
			return out;
		}
		else {
			double v0 = VectorUtil.getCoord(v, _coneIndices[0]);
			double v2 = VectorUtil.getCoord(v, _coneIndices[2]);
			double s = sqrt(v0 * v0 + v2 * v2);
			if (s > BulletGlobals.fltEpsilon) {
				double d = _radius / s;
				VectorUtil.setCoord(out, _coneIndices[0], VectorUtil.getCoord(v, _coneIndices[0]) * d);
				VectorUtil.setCoord(out, _coneIndices[1], -halfHeight);
				VectorUtil.setCoord(out, _coneIndices[2], VectorUtil.getCoord(v, _coneIndices[2]) * d);
				return out;
			} else {
				VectorUtil.setCoord(out, _coneIndices[0], 0);
				VectorUtil.setCoord(out, _coneIndices[1], -halfHeight);
				VectorUtil.setCoord(out, _coneIndices[2], 0);
				return out;
			}
		}
	}

	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		return _coneLocalSupport(vec, out);
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		for (int i=0; i<numVectors; i++) {
			Vector3 vec = vectors[i];
			_coneLocalSupport(vec, supportVerticesOut[i]);
		}
	}

	@override
	Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
		Vector3 supVertex = _coneLocalSupport(vec, out);
		if (getMargin() != 0) {
			Vector3 vecnorm = Vector3.copy(vec);
			if (vecnorm.length2 < (BulletGlobals.fltEpsilon * BulletGlobals.fltEpsilon)) {
				vecnorm.setValues(-1, -1, -1);
			}
			vecnorm.normalize();
			supVertex.scaleAdd(getMargin(), vecnorm, supVertex);
		}
		return supVertex;
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.coneShapeProxytype;
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		Transform identity = Transform();
		identity.setIdentity();
		Vector3 aabbMin = Vector3.zero(), aabbMax = Vector3.zero();
		getAabb(identity, aabbMin, aabbMax);

		Vector3 halfExtents = Vector3.zero();
		halfExtents.sub2(aabbMax,aabbMin);
		halfExtents.scale(0.5);

		double margin = getMargin();

		double lx = 2 * (halfExtents.x + margin);
		double ly = 2 * (halfExtents.y + margin);
		double lz = 2 * (halfExtents.z + margin);
		double x2 = lx * lx;
		double y2 = ly * ly;
		double z2 = lz * lz;
		double scaledmass = mass * 0.08333333;

		inertia.setValues(y2 + z2, x2 + z2, x2 + y2);
		inertia.scale(scaledmass);

		//inertia.x() = scaledmass * (y2+z2);
		//inertia.y() = scaledmass * (x2+z2);
		//inertia.z() = scaledmass * (x2+y2);
	}

	@override
	String getName() {
		return "Cone";
	}

	// choose upAxis index
	void setConeUpIndex(int upIndex) {
		switch (upIndex) {
			case 0:
				_coneIndices[0] = 1;
				_coneIndices[1] = 0;
				_coneIndices[2] = 2;
				break;

			case 1:
				_coneIndices[0] = 0;
				_coneIndices[1] = 1;
				_coneIndices[2] = 2;
				break;

			case 2:
				_coneIndices[0] = 0;
				_coneIndices[1] = 2;
				_coneIndices[2] = 1;
				break;

			default:
				assert (false);
		}
	}

	int getConeUpIndex() {
		return _coneIndices[1];
	}
	
}
