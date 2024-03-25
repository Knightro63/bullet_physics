/*
 * Dart port of Bullet (c) 2024 @Knightro
 * 
 * ShapeHull implemented by John McCutchan.
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

import 'package:bullet_physics/collision/shapes/convex_shape.dart';
import 'package:bullet_physics/linearmath/convexhull/hull_desc.dart';
import 'package:bullet_physics/linearmath/convexhull/hull_flags.dart';
import 'package:bullet_physics/linearmath/convexhull/hull_library.dart';
import 'package:bullet_physics/linearmath/convexhull/hull_result.dart';
import "package:bullet_physics/linearmath/misc_util.dart";
import 'package:bullet_physics/utils/int_array_list.dart';
import 'package:bullet_physics/utils/object_array_list.dart';
import 'package:vector_math/vector_math.dart';

/**
 * ShapeHull takes a {@link ConvexShape}, builds the convex hull using {@link HullLibrary}
 * and provides triangle indices and vertices.
 * 
 * @author jezek2
 */
class ShapeHull {
	ObjectArrayList<Vector3> vertices = ObjectArrayList();
	IntArrayList indices = IntArrayList();
	int _numIndices = 0;
	ConvexShape shape;

	ObjectArrayList<Vector3> unitSpherePoints = ObjectArrayList();

	ShapeHull(this.shape) {
		vertices.clear();
		indices.clear();

		MiscUtil.resizeObjectArray(unitSpherePoints, _numUintSpherePoints+ConvexShape.maxPreferredPenetrationDirections*2, Vector3.zero());
		for (int i=0; i<_constUnitSpherePoints.length; i++) {
			unitSpherePoints.getQuick(i)?.setFrom(_constUnitSpherePoints.getQuick(i));
		}
	}

	bool buildHull(double margin) {
		Vector3 norm = Vector3.zero();

		int numSampleDirections = _numUintSpherePoints;
		{
			int numPDA = shape.getNumPreferredPenetrationDirections();
			if (numPDA != 0) {
				for (int i=0; i<numPDA; i++) {
					shape.getPreferredPenetrationDirection(i, norm);
					unitSpherePoints.getQuick(numSampleDirections)?.setFrom(norm);
					numSampleDirections++;
				}
			}
		}

		ObjectArrayList<Vector3> supportPoints = ObjectArrayList();
		MiscUtil.resizeObjectArray(supportPoints, _numUintSpherePoints + ConvexShape.maxPreferredPenetrationDirections * 2, Vector3.zero());

		for (int i=0; i<numSampleDirections; i++) {
      if(supportPoints.getQuick(i) != null){
			  shape.localGetSupportingVertex(unitSpherePoints.getQuick(i)!, supportPoints.getQuick(i)!);
      }
		}

		HullDesc hd = HullDesc();
		hd.flags = HullFlags.triangles;
		hd.vcount = numSampleDirections;

		//#ifdef BT_USE_DOUBLE_PRECISION
		//hd.mVertices = &supportPoints[0];
		//hd.mVertexStride = sizeof(btVector3);
		//#else
		hd.vertices = supportPoints;
		//hd.vertexStride = 3 * 4;
		//#endif

		HullLibrary hl = HullLibrary();
		HullResult hr = HullResult();
		if (!hl.createConvexHull(hd, hr)) {
			return false;
		}

		MiscUtil.resizeObjectArray(vertices, hr.numOutputVertices, Vector3.zero());

		for (int i=0; i<hr.numOutputVertices; i++) {
      if(hr.outputVertices.getQuick(i) != null){
			  vertices.getQuick(i)?.setFrom(hr.outputVertices.getQuick(i)!);
      }
		}
		_numIndices = hr.numIndices;
		MiscUtil.resizeArray(indices, numIndices, 0);
		for (int i=0; i<numIndices; i++) {
			indices.set(i, hr.indices.get(i));
		}

		// free temporary hull result that we just copied
		hl.releaseResult(hr);

		return true;
	}

	int numTriangles() {
		return numIndices ~/ 3;
	}

	int numVertices() {
		return vertices.size;
	}

	int get numIndices => _numIndices;


	ObjectArrayList<Vector3> getVertexPointer() {
		return vertices;
	}

	IntArrayList getIndexPointer() {
		return indices;
	}

	////////////////////////////////////////////////////////////////////////////
	
	static const int _numUintSpherePoints = 42;
	
	static final List<Vector3> _constUnitSpherePoints = [
    Vector3(0.000000, -0.000000, -1.000000),
		Vector3(0.723608, -0.525725, -0.447219),
		Vector3(-0.276388, -0.850649, -0.447219),
		Vector3(-0.894426, -0.000000, -0.447216),
		Vector3(-0.276388, 0.850649, -0.447220),
		Vector3(0.723608, 0.525725, -0.447219),
		Vector3(0.276388, -0.850649, 0.447220),
		Vector3(-0.723608, -0.525725, 0.447219),
		Vector3(-0.723608, 0.525725, 0.447219),
		Vector3(0.276388, 0.850649, 0.447219),
		Vector3(0.894426, 0.000000, 0.447216),
		Vector3(-0.000000, 0.000000, 1.000000),
		Vector3(0.425323, -0.309011, -0.850654),
		Vector3(-0.162456, -0.499995, -0.850654),
		Vector3(0.262869, -0.809012, -0.525738),
		Vector3(0.425323, 0.309011, -0.850654),
		Vector3(0.850648, -0.000000, -0.525736),
		Vector3(-0.525730, -0.000000, -0.850652),
		Vector3(-0.688190, -0.499997, -0.525736),
		Vector3(-0.162456, 0.499995, -0.850654),
		Vector3(-0.688190, 0.499997, -0.525736),
		Vector3(0.262869, 0.809012, -0.525738),
		Vector3(0.951058, 0.309013, 0.000000),
		Vector3(0.951058, -0.309013, 0.000000),
		Vector3(0.587786, -0.809017, 0.000000),
		Vector3(0.000000, -1.000000, 0.000000),
		Vector3(-0.587786, -0.809017, 0.000000),
		Vector3(-0.951058, -0.309013, -0.000000),
		Vector3(-0.951058, 0.309013, -0.000000),
		Vector3(-0.587786, 0.809017, -0.000000),
		Vector3(-0.000000, 1.000000, -0.000000),
		Vector3(0.587786, 0.809017, -0.000000),
		Vector3(0.688190, -0.499997, 0.525736),
		Vector3(-0.262869, -0.809012, 0.525738),
		Vector3(-0.850648, 0.000000, 0.525736),
		Vector3(-0.262869, 0.809012, 0.525738),
		Vector3(0.688190, 0.499997, 0.525736),
		Vector3(0.525730, 0.000000, 0.850652),
		Vector3(0.162456, -0.499995, 0.850654),
		Vector3(-0.425323, -0.309011, 0.850654),
		Vector3(-0.425323, 0.309011, 0.850654),
		Vector3(0.162456, 0.499995, 0.850654),
  ];
}
