/*
 * Port of btHeightfieldTerrainShape by Dominic Browne <dominic.browne@hotmail.co.uk>
 *
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
import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/scalar_type.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";
import 'package:vector_math/vector_math.dart';

class HeightfieldTerrainShapeDom extends ConcaveShape {
	static const int xAxis = 0;
	static const int yAxis = 1;
	static const int zAxis = 2;

	Vector3 mLocalAabbMin = Vector3.zero();
	Vector3 mLocalAabbMax = Vector3.zero();
	Vector3 mLocalOrigin = Vector3.zero();

	// /terrain data
	int mHeightStickWidth = 0;
	int mHeightStickLength = 0;
	double mMinHeight = 0;
	double mMaxHeight = 0;
	double mWidth = 0;
	double mLength = 0;
	double mHeightScale = 0;
	late List<double> mHeightfieldDatadouble;
	ScalarType mHeightDataType = ScalarType.double;
	bool mFlipQuadEdges = false;
	bool mUseDiamondSubdivision = false;
	int mUpAxis = 0;
	Vector3 mLocalScaling = Vector3(1, 1, 1);

	HeightfieldTerrainShapeDom(
    int heightStickWidth, 
    int heightStickLength, 
    List<double> heightfieldData, 
    double heightScale, 
    double minHeight, 
    double maxHeight, 
    int upAxis, 
    bool flipQuadEdges
  ) {
		_initialize(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis, ScalarType.double, flipQuadEdges);
	}

	void _initialize(int heightStickWidth, int heightStickLength, List<double> heightfieldData, double heightScale, double minHeight, double maxHeight, int upAxis, ScalarType f, bool flipQuadEdges) {
		mHeightStickWidth = heightStickWidth;
		mHeightStickLength = heightStickLength;
		mMinHeight = minHeight*heightScale;
		mMaxHeight = maxHeight*heightScale;
		mWidth = (heightStickWidth - 1);
		mLength = (heightStickLength - 1);
		mHeightScale = heightScale;
		mHeightfieldDatadouble = heightfieldData;
		mFlipQuadEdges = flipQuadEdges;

		// determine min/max axis-aligned bounding box (aabb) values
		switch (mUpAxis) {
		case 0: {
			mLocalAabbMin.setValues(mMinHeight, 0, 0);
			mLocalAabbMax.setValues(mMaxHeight, mWidth, mLength);
			break;
		}
		case 1: {
			mLocalAabbMin.setValues(0, mMinHeight, 0);
			mLocalAabbMax.setValues(mWidth, mMaxHeight, mLength);
			break;
		}
		case 2: {
			mLocalAabbMin.setValues(0, 0, mMinHeight);
			mLocalAabbMax.setValues(mWidth, mLength, mMaxHeight);
			break;
		}
		}

		// remember origin (defined as exact middle of aabb)
		// mLocalOrigin = btScalar(0.5) * (mLocalAabbMin + mLocalAabbMax);

		mLocalOrigin.setFrom(mLocalAabbMin);
		mLocalOrigin.add(mLocalAabbMax);
		mLocalOrigin.x = mLocalOrigin.x * 0.5;
		mLocalOrigin.y = mLocalOrigin.y * 0.5;
		mLocalOrigin.z = mLocalOrigin.z * 0.5;

	}

	@override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 localAabbMin = Vector3.zero();

		localAabbMin.x = aabbMin.x * (1 / mLocalScaling.x);
		localAabbMin.y = aabbMin.y * (1 / mLocalScaling.y);
		localAabbMin.z = aabbMin.z * (1 / mLocalScaling.z);

		Vector3 localAabbMax = Vector3.zero();
		localAabbMax.x = aabbMax.x * (1 / mLocalScaling.x);
		localAabbMax.y = aabbMax.y * (1 / mLocalScaling.y);
		localAabbMax.z = aabbMax.z * (1 / mLocalScaling.z);

		localAabbMin.add(mLocalOrigin);
		localAabbMax.add(mLocalOrigin);

		// quantize the aabbMin and aabbMax, and adjust the start/end ranges
		List<int> quantizedAabbMin = [0,0,0];
		List<int> quantizedAabbMax = [0,0,0];
		_quantizeWithClamp(quantizedAabbMin, localAabbMin);
		_quantizeWithClamp(quantizedAabbMax, localAabbMax);

		// expand the min/max quantized values
		// this is to catch the case where the input aabb falls between grid points!
		for (int i = 0; i < 3; ++i) {
			quantizedAabbMin[i]--;
			quantizedAabbMax[i]++;
		}

		int startX = 0;
		int endX = mHeightStickWidth - 1;
		int startJ = 0;
		int endJ = mHeightStickLength - 1;

		switch (mUpAxis) {
		case 0: {
			if (quantizedAabbMin[1] > startX){
				startX = quantizedAabbMin[1];
      }
			if (quantizedAabbMax[1] < endX){
				endX = quantizedAabbMax[1];
      }
			if (quantizedAabbMin[2] > startJ){
				startJ = quantizedAabbMin[2];
      }
			if (quantizedAabbMax[2] < endJ){
				endJ = quantizedAabbMax[2];
      }
			break;
		}
		case 1: {
			if (quantizedAabbMin[0] > startX){
				startX = quantizedAabbMin[0];
      }
			if (quantizedAabbMax[0] < endX){
				endX = quantizedAabbMax[0];
      }
			if (quantizedAabbMin[2] > startJ){
				startJ = quantizedAabbMin[2];
      }
			if (quantizedAabbMax[2] < endJ){
				endJ = quantizedAabbMax[2];
      }
			break;
		}

		case 2: {
			if (quantizedAabbMin[0] > startX){
				startX = quantizedAabbMin[0];
      }
			if (quantizedAabbMax[0] < endX){
				endX = quantizedAabbMax[0];
      }
			if (quantizedAabbMin[1] > startJ){
				startJ = quantizedAabbMin[1];
      }
			if (quantizedAabbMax[1] < endJ){
				endJ = quantizedAabbMax[1];
      }
			break;
		}
		}

		for (int j = startJ; j < endJ; j++) {
			for (int x = startX; x < endX; x++) {
				// Vector3 vertices[3];
				List<Vector3> vertices = [Vector3.zero(),Vector3.zero(),Vector3.zero()];
				if (mFlipQuadEdges || (mUseDiamondSubdivision && (((j + x) & 1) != 0))) {// XXX
					// first triangle
					_getVertex(x, j, vertices[0]);
					_getVertex(x + 1, j, vertices[1]);
					_getVertex(x + 1, j + 1, vertices[2]);
					callback?.processTriangle(vertices, x, j);
					// callback->processTriangle(vertices,x,j);
					// second triangle
					_getVertex(x, j, vertices[0]);
					_getVertex(x + 1, j + 1, vertices[1]);
					_getVertex(x, j + 1, vertices[2]);
					// callback->processTriangle(vertices,x,j);
          callback?.processTriangle(vertices, x, j);
				} 
        else {
					// first triangle
					_getVertex(x, j, vertices[0]);
					_getVertex(x, j + 1, vertices[1]);
					_getVertex(x + 1, j, vertices[2]);
					// callback->processTriangle(vertices,x,j);
          callback?.processTriangle(vertices, x, j);
					// second triangle
					_getVertex(x + 1, j, vertices[0]);
					_getVertex(x, j + 1, vertices[1]);
					_getVertex(x + 1, j + 1, vertices[2]);
					// callback->processTriangle(vertices,x,j);
          callback?.processTriangle(vertices, x, j);
				}
			}
		}

	}

	// / this returns the vertex in bullet-local coordinates
	void _getVertex(int x, int y, Vector3 vertex) {
		double height = _getRawHeightFieldValue(x, y);

		switch (mUpAxis) {
		case 0: {
			vertex.setValues(height - mLocalOrigin.x, (-mWidth / 2.0) + x, (-mLength / 2.0) + y);
			break;
		}
		case 1: {
			vertex.setValues((-mWidth / 2.0) + x, height - mLocalOrigin.y, (-mLength / 2.0) + y);
			break;
		}

		case 2: {
			vertex.setValues((-mWidth / 2.0) + x, (-mLength / 2.0) + y, height - mLocalOrigin.z);
			break;
		}
		}

		vertex.x = vertex.x * mLocalScaling.x;
		vertex.y = vertex.y * mLocalScaling.y;
		vertex.z = vertex.z * mLocalScaling.z;
	}

	@override
	void calculateLocalInertia(double arg0, Vector3 inertia) {
		inertia.setValues(0, 0, 0);
	}

	@override
	void getAabb(Transform t, Vector3 aabbMin, Vector3 aabbMax) {
		Vector3 halfExtents = Vector3.zero();
		halfExtents.setFrom(mLocalAabbMax);
		halfExtents.sub(mLocalAabbMin);
		halfExtents.x = halfExtents.x * mLocalScaling.x * 0.5;
		halfExtents.y = halfExtents.y * mLocalScaling.y * 0.5;
		halfExtents.z = halfExtents.z * mLocalScaling.z * 0.5;

		/*Vector3 localOrigin(0, 0, 0);
		localOrigin[mUpAxis] = (mMinHeight + mMaxHeight) * 0.5; XXX
		localOrigin *= mLocalScaling;*/

		Matrix3 absB = Matrix3.copy(t.basis);
		MatrixUtil.absolute(absB);

		Vector3 tmp = Vector3.zero();

		Vector3 center = Vector3.copy(t.origin);
		Vector3 extent = Vector3.zero();
		tmp.setFrom(absB.getRow(0));
		extent.x = tmp.dot(halfExtents);
		tmp.setFrom(absB.getRow(1));
		extent.y = tmp.dot(halfExtents);
		tmp.setFrom(absB.getRow(2));
		extent.z = tmp.dot(halfExtents);

		Vector3 margin = Vector3.zero();
		margin.setValues(getMargin(), getMargin(), getMargin());
		extent.add(margin);

		aabbMin.sub2(center,extent);
		aabbMax.add2(center,extent);
	}

	@override
	Vector3 getLocalScaling(Vector3 arg0) {
		return mLocalScaling;
	}

	@override
	String getName() {
		return "Terrain";
	}

	@override
	BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.terrainShapeProxytype;
	}

	@override
	void setLocalScaling(Vector3 scaling) {
		mLocalScaling = scaling;
	}

	// / This returns the "raw" (user's initial) height, not the actual height.
	// / The actual height needs to be adjusted to be relative to the center
	// / of the heightfield's AABB.

	double _getRawHeightFieldValue(int x, int y) {
		return mHeightfieldDatadouble[(y * mHeightStickWidth) + x] * mHeightScale;
	}

	static int getQuantized(double x) {
		if (x < 0.0) {
			return (x - 0.5).toInt();
		}
		return (x + 0.5).toInt();
	}

	// / given input vector, return quantized version
	/**
	 * This routine is basically determining the gridpoint indices for a given input vector, answering the question: "which gridpoint is closest to the provided point?".
	 *
	 * "with clamp" means that we restrict the point to be in the heightfield's axis-aligned bounding box.
	 */
	void _quantizeWithClamp(List<int> out, Vector3 clampedPoint) {
		/*
		 * btVector3 clampedPoint(point); XXX
		clampedPoint.setMax(mLocalAabbMin);
		clampedPoint.setMin(mLocalAabbMax);

		 * clampedPoint.clampMax(mLocalAabbMax,);
		clampedPoint.clampMax(mLocalAabbMax);
		clampedPoint.clampMax(mLocalAabbMax);

		clampedPoint.clampMin(mLocalAabbMin);
		clampedPoint.clampMin(mLocalAabbMin); ///CLAMPS
		clampedPoint.clampMin(mLocalAabbMin);*/

		out[0] = getQuantized(clampedPoint.x);
		out[1] = getQuantized(clampedPoint.y);
		out[2] = getQuantized(clampedPoint.z);
	}
}
