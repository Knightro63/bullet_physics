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



import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/collision/shapes/convex_shape.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

/**
 * UniformScalingShape allows to re-use uniform scaled instances of {@link ConvexShape}
 * in a memory efficient way. Istead of using {@link UniformScalingShape}, it is better
 * to use the non-uniform setLocalScaling method on convex shapes that implement it.
 * 
 * @author jezek2
 */
class UniformScalingShape extends ConvexShape {

	late ConvexShape _childConvexShape;
	late double _uniformScalingFactor;

	UniformScalingShape(ConvexShape convexChildShape, double uniformScalingFactor) {
		_childConvexShape = convexChildShape;
		_uniformScalingFactor = uniformScalingFactor;
	}

	double getUniformScalingFactor() {
		return _uniformScalingFactor;
	}

	ConvexShape getChildShape() {
		return _childConvexShape;
	}
	
	@override
	Vector3 localGetSupportingVertex(Vector3 vec, Vector3 out) {
		_childConvexShape.localGetSupportingVertex(vec, out);
		out.scale(_uniformScalingFactor);
		return out;
	}

	@override
	Vector3 localGetSupportingVertexWithoutMargin(Vector3 vec, Vector3 out) {
		_childConvexShape.localGetSupportingVertexWithoutMargin(vec, out);
		out.scale(_uniformScalingFactor);
		return out;
	}

	@override
	void batchedUnitVectorGetSupportingVertexWithoutMargin(List<Vector3> vectors, List<Vector3> supportVerticesOut, int numVectors) {
		_childConvexShape.batchedUnitVectorGetSupportingVertexWithoutMargin(vectors, supportVerticesOut, numVectors);
		for (int i=0; i<numVectors; i++) {
			supportVerticesOut[i].scale(_uniformScalingFactor);
		}
	}

	@override
	void getAabbSlow(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		_childConvexShape.getAabbSlow(t, aabbMin, aabbMax);
		Vector3 aabbCenter = Vector3.zero();
		aabbCenter.add2(aabbMax, aabbMin);
		aabbCenter.scale(0.5);

		Vector3 scaledAabbHalfExtends = Vector3.zero();
		scaledAabbHalfExtends.sub2(aabbMax, aabbMin);
		scaledAabbHalfExtends.scale(0.5 * _uniformScalingFactor);

		aabbMin.sub2(aabbCenter, scaledAabbHalfExtends);
		aabbMax.add2(aabbCenter, scaledAabbHalfExtends);
	}

	@override
	void setLocalScaling(Vector3 scaling) {
		_childConvexShape.setLocalScaling(scaling);
	}

	@override
	Vector3 getLocalScaling(Vector3 out) {
		_childConvexShape.getLocalScaling(out);
		return out;
	}

	@override
	void setMargin(double margin) {
		_childConvexShape.setMargin(margin);
	}

	@override
	double getMargin() {
		return _childConvexShape.getMargin() * _uniformScalingFactor;
	}

	@override
	int getNumPreferredPenetrationDirections() {
		return _childConvexShape.getNumPreferredPenetrationDirections();
	}

	@override
	void getPreferredPenetrationDirection(int index, Vector3 penetrationVector) {
		_childConvexShape.getPreferredPenetrationDirection(index, penetrationVector);
	}

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		_childConvexShape.getAabb(t, aabbMin, aabbMax);
		Vector3 aabbCenter = Vector3.zero();
		aabbCenter.add2(aabbMax, aabbMin);
		aabbCenter.scale(0.5);

		Vector3 scaledAabbHalfExtends = Vector3.zero();
		scaledAabbHalfExtends.sub2(aabbMax, aabbMin);
		scaledAabbHalfExtends.scale(0.5 * _uniformScalingFactor);

		aabbMin.sub2(aabbCenter, scaledAabbHalfExtends);
		aabbMax.add2(aabbCenter, scaledAabbHalfExtends);
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.uniformScalingShapeProxytype;
	}

	@override
	void calculateLocalInertia(double mass, Vector3 inertia) {
		// this linear upscaling is not realistic, but we don't deal with large mass ratios...
		_childConvexShape.calculateLocalInertia(mass, inertia);
		inertia.scale(_uniformScalingFactor);
	}

	@override
	String getName() {
		return "UniformScalingShape";
	}

}
