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

import 'package:bullet_physics/linearmath/aabb.dart';
import 'package:vector_math/vector_math.dart';

class BvhDataArray {

	int _size = 0;
	
	List<double> bound = [];
	List<int> data = [];

	int get length => _size;


	void resize(int? newSize) {
    if(newSize == null) return;
		List<double> newBound = bound.sublist(0);//List.filled(newSize*6, 0);
		List<int> newData = data.sublist(0);//List.filled(newSize, 0);
		
		// System.arraycopy(bound, 0, newBound, 0, size*6);
		// System.arraycopy(data, 0, newData, 0, size);
		
		bound = newBound;
		data = newData;
		
		_size = newSize;
	}
	
	void swap(int idx1, int idx2) {
		int pos1 = idx1*6;
		int pos2 = idx2*6;
		
		double b0 = bound[pos1+0];
		double b1 = bound[pos1+1];
		double b2 = bound[pos1+2];
		double b3 = bound[pos1+3];
		double b4 = bound[pos1+4];
		double b5 = bound[pos1+5];
		int d = data[idx1];
		
		bound[pos1+0] = bound[pos2+0];
		bound[pos1+1] = bound[pos2+1];
		bound[pos1+2] = bound[pos2+2];
		bound[pos1+3] = bound[pos2+3];
		bound[pos1+4] = bound[pos2+4];
		bound[pos1+5] = bound[pos2+5];
		data[idx1] = data[idx2];

		bound[pos2+0] = b0;
		bound[pos2+1] = b1;
		bound[pos2+2] = b2;
		bound[pos2+3] = b3;
		bound[pos2+4] = b4;
		bound[pos2+5] = b5;
		data[idx2] = d;
	}
	
	AABB getBound(int idx, AABB out) {
		int pos = idx*6;
		out.min.setValues(bound[pos+0], bound[pos+1], bound[pos+2]);
		out.max.setValues(bound[pos+3], bound[pos+4], bound[pos+5]);
		return out;
	}

	Vector3 getBoundMin(int idx, Vector3 out) {
		int pos = idx*6;
		out.setValues(bound[pos+0], bound[pos+1], bound[pos+2]);
		return out;
	}

	Vector3 getBoundMax(int idx, Vector3 out) {
		int pos = idx*6;
		out.setValues(bound[pos+3], bound[pos+4], bound[pos+5]);
		return out;
	}
	
	void setBound(int idx, AABB aabb) {
		int pos = idx*6;
		bound[pos+0] = aabb.min.x;
		bound[pos+1] = aabb.min.y;
		bound[pos+2] = aabb.min.z;
		bound[pos+3] = aabb.max.x;
		bound[pos+4] = aabb.max.y;
		bound[pos+5] = aabb.max.z;
	}
	
	int getData(int idx) {
		return data[idx];
	}
	
	void setData(int idx, int value) {
		data[idx] = value;
	}
	
}
