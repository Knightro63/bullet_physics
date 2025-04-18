/*
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
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

import "dart:typed_data";
import "package:bullet_physics/collision/shapes/concave_shape.dart";
import "package:bullet_physics/collision/shapes/triangle_callback.dart";
import 'package:vector_math/vector_math.dart';
import "package:bullet_physics/collision/broadphase/broadphase_native_type.dart";
import "package:bullet_physics/linearmath/matrix_util.dart";
import "package:bullet_physics/linearmath/transform.dart";
import "package:bullet_physics/linearmath/vector_util.dart";

enum PhyScalarType{double, uchar, int}

class HeightfieldTerrainShape extends ConcaveShape{
	late Vector3 mLocalAabbMin;
	late Vector3 mLocalAabbMax;
	late Vector3 mLocalOrigin;
	late Vector3 mLocalScaling;

	///terrain data
	int mHeightStickWidth = 0;
	int mHeightStickLength = 0;
	double mMinHeight = 0;
	double mMaxHeight = 0;
	double mWidth = 0;
	double mLength = 0;
	double mheightScale = 0;

	Uint8List? mHeightFieldDataByte;
	List<double>? mHeightFieldDatadouble;

	late PhyScalarType mHeightDataType;
	bool mFlipQuadEdges = false;
	bool mUseDiamondSubdivision = false;

	int mUpAxis = 0;

	/// preferred constructor
	/**
	  This constructor supports a range of heightfield
	  data types, and allows for a non-zero minimum height value.
	  heightScale is needed for any integer-based heightfield data types.
	 */
	HeightfieldTerrainShape(
    int heightStickWidth, 
    int heightStickLength, 
    Uint8List heightfieldData,
		double heightScale, 
    double minHeight, 
    double maxHeight, 
    int upAxis, 
    PhyScalarType heightDataType,
		bool flipQuadEdges
  ){
		initialize(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis,heightDataType, flipQuadEdges);
	}

	/// legacy constructor
	/**
	  The legacy constructor assumes the heightfield has a minimum height
	  of zero.  Only unsigned char or doubles are supported.  For legacy
	  compatibility reasons, heightScale is calculated as maxHeight / 65535 
	  (and is only used when usedoubleData = false).
	 */
	HeightfieldTerrainShape.noPhyScalarType(
    int heightStickWidth, 
    int heightStickLength, 
    Uint8List heightfieldData,
    double maxHeight, 
    int upAxis, 
    bool usedoubleData, 
    bool flipQuadEdges
  ){
		// legacy constructor: support only double or unsigned char,
		// 	and min height is zero
		PhyScalarType hdt = (usedoubleData) ? PhyScalarType.double : PhyScalarType.uchar;
		double minHeight = 0.0;
		// previously, height = uchar * maxHeight / 65535.
		// So to preserve legacy behavior, heightScale = maxHeight / 65535
		double heightScale = maxHeight / 65535;
		initialize(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis,hdt, flipQuadEdges);
	}

	HeightfieldTerrainShape.fromListDouble(
    int heightStickWidth, 
    int heightStickLength, 
    List<double> heightfieldData,
		double heightScale, 
    double minHeight, 
    double maxHeight, 
    int upAxis, 
    bool flipQuadEdges
  ){
		initialize(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis,
				PhyScalarType.double, flipQuadEdges);
	}

	double getRawHeightFieldValue(int x, int y){
		double val = 0;
		switch (mHeightDataType){
      case PhyScalarType.double:{
        if (mHeightFieldDatadouble != null){
          // double offset (4 for sizeof)
          int index = ((y * mHeightStickWidth) + x);
          val = mHeightFieldDatadouble![index];
        }
        else{
          // FIXME - MAN - provide a way of handling different data types
          // double offset (4 for sizeof)
          //int index = ((y * mHeightStickWidth) + x) * 4;
          //val = 0;//BitConverter.ToSingle(mHeightFieldDataByte, index);

          //int size = 4;
          Float32List? bb = mHeightFieldDataByte?.buffer.asFloat32List();//ByteBuffer().allocate(size).put(mHeightFieldDataByte, index, size);
          val = bb?[0] ?? 0;
          break;
        }
      }
      case PhyScalarType.uchar:{
        int heightFieldValue = mHeightFieldDataByte?[(y * mHeightStickWidth) + x] ?? 0;
        val = heightFieldValue * mheightScale;
        break;
      }
      case PhyScalarType.int:{
        // FIXME - MAN - provide a way of handling different data types
        //int index = ((y * mHeightStickWidth) + x) * 2;
        int hfValue = 0;//BitConverter.ToInt16(mHeightFieldDataByte, index);
        val = hfValue * mheightScale;
        break;
      }
		}

		return val;
	}

	void quantizeWithClamp(List<int> output, Vector3 point, int isMax){
		/// given input vector, return quantized version
		/**
		  This routine is basically determining the gridpoint indices for a given
		  input vector, answering the question: "which gridpoint is closest to the
		  provided point?".

		  "with clamp" means that we restrict the point to be in the heightfield's
		  axis-aligned bounding box.
		 */

		Vector3 clampedPoint = Vector3.zero();
		clampedPoint.setValues(point.x, point.y, point.z);
		VectorUtil.setMax(clampedPoint, mLocalAabbMin);
		VectorUtil.setMin(clampedPoint, mLocalAabbMax);

		output[0] = getQuantized(clampedPoint.x);
		output[1] = getQuantized(clampedPoint.y);
		output[2] = getQuantized(clampedPoint.z);
	}

	static int getQuantized(double x){
		if (x < 0.0){
			return (x - 0.5).toInt();
		}
		return (x + 0.5).toInt();
	}

	void getVertex(int x, int y, Vector3 vertex){
		assert (x >= 0);
		assert (y >= 0);
		assert (x < mHeightStickWidth);
		assert (y < mHeightStickLength);

		double height = getRawHeightFieldValue(x, y);

		switch (mUpAxis){
      case 0:{
        vertex.setValues(height - mLocalOrigin.x, (-mWidth / 2) + x, (-mLength / 2) + y);
        break;
      }
      case 1:{
        vertex.setValues((-mWidth / 2) + x, height - mLocalOrigin.y, (-mLength / 2) + y);
        break;
      }
      case 2:{
        vertex.setValues((-mWidth / 2) + x, (-mLength / 2) + y, height - mLocalOrigin.z);
        break;
      }
      default:{
        //need to get valid mUpAxis
        assert (false);
        vertex.setValues(0, 0, 0);
        break;
      }
		}

		VectorUtil.mul(vertex, vertex, mLocalScaling);
	}
  @override
	BroadphaseNativeType getShapeType(){
		return BroadphaseNativeType.terrainShapeProxytype;
	}

	/// initialization
	/**
	  Handles the work of constructors so that constructors can be
	  backwards-compatible without a lot of copy/paste.
	 */
	void initialize(
    int heightStickWidth, 
    int heightStickLength, 
    Object? heightfieldData, 
    double heightScale,
		double minHeight, 
    double maxHeight, 
    int upAxis, 
    PhyScalarType hdt, 
    bool flipQuadEdges
  ){
		// validation
		assert (heightStickWidth > 1, "bad width");
		assert (heightStickLength > 1, "bad length");
		assert (heightfieldData != null, "null heightfield data");
		// assert(heightScale) -- do we care?  Trust caller here
		assert (minHeight <= maxHeight, "bad min/max height");
		assert (upAxis >= 0 && upAxis < 3, "bad upAxis--should be in range [0,2]");
		assert (hdt != PhyScalarType.uchar || hdt != PhyScalarType.double || hdt != PhyScalarType.int, "Bad height data type enum");

		// initialize member variables

		mHeightStickWidth = heightStickWidth;
		mHeightStickLength = heightStickLength;
		mMinHeight = minHeight;
		mMaxHeight = maxHeight;
		mWidth = (heightStickWidth - 1);
		mLength = (heightStickLength - 1);
		mheightScale = heightScale;
		// copy the data in 
		if (heightfieldData is Uint8List){
			mHeightFieldDataByte = heightfieldData;
		}
		else if(heightfieldData is List<double>){
			mHeightFieldDatadouble = heightfieldData;
		}
		mHeightDataType = hdt;

		mFlipQuadEdges = flipQuadEdges;
		mUseDiamondSubdivision = false;
		mUpAxis = upAxis;

		mLocalScaling = Vector3.zero();
		mLocalScaling.setValues(1, 1, 1);
		// determine min/max axis-aligned bounding box (aabb) values

		mLocalAabbMin = Vector3.zero();
		mLocalAabbMax = Vector3.zero();
		switch (mUpAxis){
      case 0:{
        mLocalAabbMin.setValues(mMinHeight, 0, 0);
        mLocalAabbMax.setValues(mMaxHeight, mWidth, mLength);
        break;
      }
      case 1:{
        mLocalAabbMin.setValues(0, mMinHeight, 0);
        mLocalAabbMax.setValues(mWidth, mMaxHeight, mLength);
        break;
      }
      case 2:{
        mLocalAabbMin.setValues(0, 0, mMinHeight);
        mLocalAabbMax.setValues(mWidth, mLength, mMaxHeight);
        break;
      }
      default:{
        //need to get valid mUpAxis
        throw "Bad mUpAxis";
      }
		}

		// remember origin (defined as exact middle of aabb)
		mLocalOrigin = Vector3.zero();
		mLocalOrigin.setValues(0, 0, 0);
		VectorUtil.add(mLocalOrigin, mLocalAabbMin, mLocalAabbMax);
		VectorUtil.scale(mLocalOrigin, mLocalOrigin, 0.5);

		for (int i = 0; i < vertices.length; ++i){
			vertices[i] = Vector3.zero();
		}

	}

	void setUseDiamondSubdivision(bool useDiamondSubdivision){
		mUseDiamondSubdivision = useDiamondSubdivision;
	}

	@override
	void getAabb(Transform trans, Vector3 aabbMin, Vector3 aabbMax){
		Vector3 tmp = Vector3.zero();

		Vector3 localHalfExtents = Vector3.zero();
		localHalfExtents.sub2(mLocalAabbMax, mLocalAabbMin);
		VectorUtil.mul(localHalfExtents,localHalfExtents,mLocalScaling);
		//localHalfExtents.mul(localHalfExtents,mLocalScaling);
		localHalfExtents.scale(0.5);

		Vector3 localOrigin = Vector3.zero();
		localOrigin.setValues(0,0,0);
		VectorUtil.setCoord(localOrigin,mUpAxis,(mMinHeight + mMaxHeight)*0.5);
		VectorUtil.mul(localOrigin,localOrigin,mLocalScaling);
		
		Matrix3 absB = Matrix3.copy(trans.basis);
		MatrixUtil.absolute(absB);

		Vector3 center = Vector3.copy(trans.origin);
		Vector3 extent = Vector3.zero();
		absB.getRowWith(0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		absB.getRowWith(1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		absB.getRowWith(2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		Vector3 margin = Vector3.zero();
		margin.setValues(getMargin(), getMargin(), getMargin());
		extent.add(margin);

		aabbMin.sub2(center, extent);
		aabbMax.add2(center, extent);
	}

	/// process all triangles within the provided axis-aligned bounding box
	/**
	  basic algorithm:
	    - convert input aabb to local coordinates (scale down and shift for local origin)
	    - convert input aabb to a range of heightfield grid points (quantize)
	    - iterate over all triangles in that subset of the grid
	 */
	//quantize the aabbMin and aabbMax, and adjust the start/end ranges
	List<int> quantizedAabbMin = [0,0,0];
	List<int> quantizedAabbMax = [0,0,0];
	List<Vector3> vertices = [Vector3.zero(),Vector3.zero(),Vector3.zero()];

	void checkNormal(List<Vector3> vertices1, TriangleCallback? callback){

		Vector3 tmp1 = Vector3.zero();
		Vector3 tmp2 = Vector3.zero();
		Vector3 normal = Vector3.zero();

		tmp1.sub2(vertices1[1], vertices1[0]);
		tmp2.sub2(vertices1[2], vertices1[0]);

		normal.cross2(tmp1, tmp2);
		normal.normalize();
	}
  @override
	void processAllTriangles(TriangleCallback? callback, Vector3 aabbMin, Vector3 aabbMax){
		// scale down the input aabb's so they are in local (non-scaled) coordinates
		Vector3 invScale = Vector3.zero();
		invScale.setValues(1 / mLocalScaling.x, 1 / mLocalScaling.y, 1 / mLocalScaling.z);

		Vector3 localAabbMin = Vector3.zero();
		Vector3 localAabbMax = Vector3.zero();

		VectorUtil.mul(localAabbMin, aabbMin, invScale);
		VectorUtil.mul(localAabbMax, aabbMax, invScale);

		// account for local origin
		VectorUtil.add(localAabbMin, localAabbMin, mLocalOrigin);
		VectorUtil.add(localAabbMax, localAabbMax, mLocalOrigin);

		quantizeWithClamp(quantizedAabbMin, localAabbMin, 0);
		quantizeWithClamp(quantizedAabbMax, localAabbMax, 1);

		// expand the min/max quantized values
		// this is to catch the case where the input aabb falls between grid points!
		for (int i = 0; i < 3; ++i){
			quantizedAabbMin[i]--;
			quantizedAabbMax[i]++;
		}

		int startX = 0;
		int endX = mHeightStickWidth - 1;
		int startJ = 0;
		int endJ = mHeightStickLength - 1;

		switch (mUpAxis){
      case 0:{
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
      case 1:{
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
      case 2:{
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
      default:{
        //need to get valid mUpAxis
        assert (false);
        break;
      }
		}

		// debug draw the boxes?
		for (int j = startJ; j < endJ; j++){
			for (int x = startX; x < endX; x++){
				if (mFlipQuadEdges || (mUseDiamondSubdivision && (((j + x) & 1) > 0))){
					//first triangle
					getVertex(x, j, vertices[0]);
					getVertex(x + 1, j, vertices[1]);
					getVertex(x + 1, j + 1, vertices[2]);
					callback?.processTriangle(vertices, x, j);
					//second triangle
					getVertex(x, j, vertices[0]);
					getVertex(x + 1, j + 1, vertices[1]);
					getVertex(x, j + 1, vertices[2]);

					callback?.processTriangle(vertices, x, j);
				}
				else{
					//first triangle
					getVertex(x, j, vertices[0]);
					getVertex(x, j + 1, vertices[1]);
					getVertex(x + 1, j, vertices[2]);
					checkNormal(vertices, callback);
					callback?.processTriangle(vertices, x, j);

					//second triangle
					getVertex(x + 1, j, vertices[0]);
					getVertex(x, j + 1, vertices[1]);
					getVertex(x + 1, j + 1, vertices[2]);
					checkNormal(vertices, callback);
					callback?.processTriangle(vertices, x, j);
				}
			}
		}
	}
  @override
	void calculateLocalInertia(double mass, Vector3 inertia){
		//moving concave objects not supported
		inertia.setValues(0, 0, 0);
	}
  @override
	void setLocalScaling(Vector3 scaling){
		mLocalScaling.setFrom(scaling);
	}
  @override
	Vector3 getLocalScaling(Vector3 localScaling){
		localScaling.setFrom(mLocalScaling);
		return localScaling;
	}

	//debugging
  @override
	String getName(){
		return "HEIGHTFIELD";
	}
}
